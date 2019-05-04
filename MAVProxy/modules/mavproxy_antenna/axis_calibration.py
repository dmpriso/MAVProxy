class CalibrationError(Exception):
    def __init__(self, descr):
        self.descr = descr

    def __str__(self):
        return f'Invalid calibration: {self.descr}'

def sign(x):
    return (x > 0) - (x < 0)

def avg(list):
    return sum(list) / len(list)

def angleDiff360(fromAngle, toAngle):
    '''Calculates the angle between fromAngle and toAngle, taking rotation (wrap-around after 360) into account.'''
    diff = float(toAngle) - float(fromAngle)
    if abs(diff) <= 180.0:
        return diff
    # let's assume 350 ... 10
    return angleDiff360(fromAngle + 360.0 * float(sign(diff)), toAngle)

class PwmHelper:
    def __init__(self, axis, degrees):
        self.axis = axis
        self.set_degrees(degrees)

    def set_degrees(self, degrees):
        self.degrees = degrees
        self.pwm = self.axis._degrees_to_pwm(self.degrees)

    def rotate360(self, direction):
        self.set_degrees(self.degrees + 360.0 * float(self.axis._direction) * float(direction))

    def is_too_low(self):
        return self.pwm < self.axis._min_pwm

    def is_too_high(self):
        return self.pwm > self.axis._max_pwm


class AxisCalibration:
    def __init__(self):
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
        self._calib_status = ''

    def set_min_pwm(self, pwm):
        '''Sets the minimum PWM value where the tracker may slew to'''
        self._min_pwm = pwm
        self.calc_calibration()

    def set_max_pwm(self, pwm):
        '''Sets the maximum PWM value where the tracker may slew to'''
        self._max_pwm = pwm
        self.calc_calibration()

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
    
    def _prepare_positions_and_calc_diffs(self):
        '''First step of calibration calculation. Sorts the differences between the stored positions'''
        self._positions = {pwm : self._positions[pwm] for pwm in sorted(self._positions)}
        self._clear_direction()
        last_pwm = next(iter(self._positions))

        positions = {}
        diffs = []
        last_pwm = None

        for pwm in sorted(self._positions.keys()):
            if last_pwm == None:
                last_pwm = pwm
                last_degrees = self._positions[last_pwm]
                positions[last_pwm] = last_degrees
                continue

            degrees = self._positions[pwm]

            diff_pwm = pwm - last_pwm
            diff_degrees = angleDiff360(last_degrees, degrees)
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
        if self._min_pwm == None:
            self._min_pwm = min(self._positions.keys())
        if self._max_pwm == None:
            self._max_pwm = max(self._positions.keys())

        self._min_degrees = self._pwm_to_degrees(self._min_pwm)
        self._max_degrees = self._pwm_to_degrees(self._max_pwm)

    def _calc_factor_and_offset(self):
        factors = [diff_pwm / diff_degrees for (diff_pwm, diff_degrees) in self._diffs]
        self._factor = avg(factors)
        
        offsets = [pwm - self._positions[pwm] * self._factor for pwm in self._positions]
        self._offset = avg(offsets)


    def calc_calibration(self):
        '''Performs the actual calibration after data points have been added.'''
        self._is_calibrated = False
        self._calib_status = 'Calibrating'

        if self._prepare_positions_and_calc_diffs() == False:
            return
        
        self._calc_factor_and_offset()
        self._set_minmax()

        self._calib_status = 'Calibrated'
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

        return positions
        
        
if __name__ == '__main__':
    diff = angleDiff360(10, 20)
    assert diff == 10
    diff = angleDiff360(20, 10)
    assert diff == -10
    diff = angleDiff360(10, 350)
    assert diff == -20
    diff = angleDiff360(350, 10)
    assert diff == 20

    calib = AxisCalibration()
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


