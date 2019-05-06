from MAVProxy.modules.mavproxy_servotracker.calc_tools import sign, avg, angle_diff, normalize_angle

class CalibrationError(Exception):
    def __init__(self, descr):
        self.descr = descr

    def __str__(self):
        return f'Invalid calibration: {self.descr}'

class PwmHelper:
    def __init__(self, axis, degrees):
        self._axis = axis
        self.set_degrees(degrees)
        self._nearest_in_range = None
        self._smallest_diff = None

    def set_degrees(self, degrees):
        self.degrees = degrees
        self.pwm = self._axis._degrees_to_pwm(self.degrees)

    def rotate360(self, direction):
        self.set_degrees(self.degrees + 360.0 * float(self._axis._direction) * float(direction))

    def _try_set_nearest(self, limit):
        diff = abs(self.pwm - limit)
        if self._smallest_diff is None or self._smallest_diff > diff:
            self._smallest_diff = diff
            self._nearest_in_range = limit

    def is_too_low(self):
        if self.pwm < self._axis._min_pwm:
            self._try_set_nearest(self._axis._min_pwm)
            return True
        return False
        
    def is_too_high(self):
        if self.pwm > self._axis._max_pwm:
            self._try_set_nearest(self._axis._max_pwm)
            return True
        return False

    def get_nearest_in_range(self):
        return self._nearest_in_range

class AxisCalibration:
    def __init__(self, desired_range):
        self._desired_range = desired_range
        self.clear()

    def add_calibration_point(self, degrees, pwm):
        '''Associates a PWM value with a heading value in degrees >=0 and < 360.'''
        degrees = float(degrees)
        pwm = int(pwm)

        if degrees < 0 or degrees >= 360:
            raise ValueError(degrees)
        elif pwm < 600 or pwm > 2400:
            raise ValueError(pwm)

        self._positions[pwm] = degrees

    def clear(self):
        '''Clears the calibration data.'''
        self._positions = {}
        self._is_calibrated = False
        self._min_pwm = None
        self._max_pwm = None
        self._calib_status = 'Uncalibrated'

    def set_min_pwm(self, pwm):
        '''Sets the minimum PWM value where the tracker may slew to'''
        self._min_pwm = pwm

    def set_max_pwm(self, pwm):
        '''Sets the maximum PWM value where the tracker may slew to'''
        self._max_pwm = pwm

    def is_calibrated(self):
        return self._is_calibrated

    def calib_status_descr(self):
        return self._calib_status

    def _pwm_to_degrees(self, pwm):
        return (pwm - self._offset) / self._factor

    def _degrees_to_pwm(self, degrees):
        return degrees * self._factor + self._offset

    def _clear_direction(self):
        self._direction = None

    def _set_direction(self, direction):
        if (self._direction == None):
            self._direction = direction
            return True

        if not self._direction == None and not self._direction == direction:
            self._calib_status = 'Could not estimate direction of rotation. Make sure calibration points are <180 deg apart'
            return False

        return True

    def _normalize_positions(self):
        '''Tries to arrange the positions in the normal 0...360 range, even if the tracker direction is reverse.'''
        min_degrees = min(self._positions.values())
        max_degrees = max(self._positions.values())
        mid = avg([min_degrees, max_degrees])
        # try to shift them to a normal range
        shift = normalize_angle(mid) - mid
        self._positions = {pwm : (degrees + shift) for (pwm, degrees) in self._positions.items()}

    
    def _prepare_positions_and_calc_diffs(self):
        '''First step of calibration calculation. Sorts the differences between the stored positions'''
        self._positions = {pwm : self._positions[pwm] for pwm in sorted(self._positions)}
        self._clear_direction()
        last_pwm = next(iter(self._positions))

        positions = {}
        diffs = []
        last_pwm = None

        if len(self._positions) <= 1:
            return False

        for pwm in sorted(self._positions.keys()):
            if last_pwm == None:
                last_pwm = pwm
                last_degrees = self._positions[last_pwm]
                positions[last_pwm] = last_degrees
                continue

            degrees = self._positions[pwm]

            diff_pwm = pwm - last_pwm
            diff_degrees = angle_diff(last_degrees, degrees)
            assert abs(diff_degrees) <= 180.0

            degrees = last_degrees + diff_degrees

            if (abs(diff_degrees) <= 0.1):
                self._calib_status = 'Data points must be apart by at least 0.1 degree and at most 180 degrees'
                return False
            degrees_direction = sign(diff_degrees)

            self._set_direction(degrees_direction)

            positions[pwm] = degrees
            diffs.append((diff_pwm, diff_degrees))

            last_pwm = pwm
            last_degrees = degrees

        self._positions = positions
        self._diffs = diffs

        return True

    def _set_minmax(self):
        filter = lambda l: [i for i in l if i is not None]

        pwms = [key for key in self._positions]
        self._min_pwm = min(pwms + filter([self._min_pwm]))
        self._max_pwm = max(pwms + filter([self._max_pwm]))

        self._min_degrees = self._pwm_to_degrees(self._min_pwm)
        self._max_degrees = self._pwm_to_degrees(self._max_pwm)

    def _calc_factor_and_offset(self):
        factors = [diff_pwm / diff_degrees for (diff_pwm, diff_degrees) in self._diffs]
        self._factor = avg(factors)
        
        offsets = [pwm - self._positions[pwm] * self._factor for pwm in self._positions]
        self._offset = avg(offsets)

    def _get_calibration_properties(self):
        return {
            'Degrees->PWM Factor': self._factor,
            'Reverse': self._factor < 0,
            'PWM offset': self._offset,
            'Minimum PWM': self._min_pwm,
            'Maximum PWM': self._max_pwm,
            'Minimum degrees': self._min_degrees,
            'Maximum degrees': self._max_degrees,
            'Total movable angle': abs(self._max_degrees - self._min_degrees),
            'Calibration points': len(self._positions)
        }

    def _get_calibration_point_error(self, pwm):
        degrees = self._positions[pwm]
        calculated_degrees = self._pwm_to_degrees(pwm)
        diff = angle_diff(degrees, calculated_degrees)
        return diff

    def _get_calibration_point_descriptions(self):
        return {f'{pwm}->{degrees} error': f'{self._get_calibration_point_error(pwm):0.2f} deg' \
            for (pwm, degrees) in self._positions.items()}


    def get_calibration_details(self):
        lines = \
            [ 'Calibration properties:' ] + \
            [ '{0:20} {1}'.format(key, value) for (key, value) in self._get_calibration_properties().items() ] + \
            [ '{0:20} {1}'.format(key, value) for (key, value) in self._get_calibration_point_descriptions().items() ]
        return '\n'.join(lines) + '\n'

    def get_calibration_warnings(self):
        errors = [ (pwm, degrees, self._get_calibration_point_error(pwm)) for (pwm, degrees) in self._positions.items() ]
        filtered = [ f'WARNING: Error of {err} at calibration point {pwm}->{degrees}' for (pwm, degrees, err) in errors if abs(err) > 20 ]
        
        range = abs(self._max_degrees - self._min_degrees)
        if self._desired_range is not None and range < self._desired_range:
            filtered = filtered + [f'WARNING: Calibrated range of axis ({range} degrees) is smaller than desired ({self._desired_range} degrees)']
        
        return '\n'.join(filtered)

    def calc_calibration(self):
        '''Performs the actual calibration after data points have been added.'''
        self._is_calibrated = False
        self._calib_status = 'Calibrating'

        if self._prepare_positions_and_calc_diffs() == False:
            return
        
        self._normalize_positions()
        self._calc_factor_and_offset()
        self._set_minmax()

        self._calib_status = f'Calibrated.'
        warnings = self.get_calibration_warnings()
        if warnings is not None and not warnings == '':
            self._calib_status = self._calib_status + '\n' + warnings

        self._is_calibrated = True

    def degrees_to_pwm_positions(self, degrees):
        degrees = float(degrees)

        if degrees < 0 or degrees >= 360:
            raise ValueError(degrees)

        if self._is_calibrated == False:
            raise CalibrationError('Not calibrated yet')

        helper = PwmHelper(self, degrees)
        while helper.is_too_low() == False:
            helper.rotate360(-1)

        while helper.is_too_low() == True:
            helper.rotate360(1)

        positions = []
        while helper.is_too_high() == False:
            positions.append(helper.pwm)
            helper.rotate360(1)

        if len(positions) == 0:
            # if no position could be found, return the nearest miss
            # this will be the best position "in range"
            return [helper.get_nearest_in_range()]

        return positions
        
        
if __name__ == '__main__':
    diff = angle_diff(10, 20)
    assert diff == 10
    diff = angle_diff(20, 10)
    assert diff == -10
    diff = angle_diff(10, 350)
    assert diff == -20
    diff = angle_diff(350, 10)
    assert diff == 20

    calib = AxisCalibration(360)
    # very simple test case
    calib.add_calibration_point(0, 1000)
    calib.add_calibration_point(90, 1250)
    calib.add_calibration_point(180, 1500)
    calib.add_calibration_point(270, 1750)
    calib.add_calibration_point(0, 2000)
    calib.set_min_pwm(1000)
    calib.set_max_pwm(2000)

    if calib.calc_calibration() == False:
        raise AssertionError()
    
    positions = calib.degrees_to_pwm_positions(180)
    if not len(positions) == 1 or not positions[0] == 1500:
        raise AssertionError()

    positions = calib.degrees_to_pwm_positions(0)
    if not positions == [1000, 2000]:
        raise AssertionError()
        
    calib.clear()
    # reverse direction and a bit of overlap
    calib.add_calibration_point(10, 970)
    calib.add_calibration_point(270, 1250)
    calib.add_calibration_point(180, 1500)
    calib.add_calibration_point(90, 1750)
    calib.add_calibration_point(350, 2030)

    if calib.calc_calibration() == False:
        raise AssertionError()

    positions = calib.degrees_to_pwm_positions(180)
    if not len(positions) == 1 or not positions[0] == 1500:
        raise AssertionError()


