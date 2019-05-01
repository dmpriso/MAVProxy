from tracking_antenna import TrackingAntenna
from servo_controller import ServoController

class CalibrationError(Exception):
    def __init__(self, descr):
        self.descr = descr

    def __str__(self):
        return f'Invalid calibration: {self.descr}'

class AxisCalibration:
    def __init__(self):
        self.clear()

    def add_calibration_point(self, degrees, pwm):
        if degrees < 0 or degrees >= 360:
            raise CalibrationError(f'The degree value {degrees} is outside the range 0...<360')
        self._positions[degrees] = pwm

    def clear(self):
        self._positions = {}
        self._is_calibrated = False
        self._min_pwm = 1000
        self._max_pwm = 2000

    def set_min_pwm(self, pwm):
        self._min_pwm = pwm
        self.calc_calibration()

    def set_max_pwm(self, pwm):
        self._max_pwm = pwm
        self.calc_calibration()

    def is_calibrated(self):
        return self._is_calibrated

    def _pwm_to_degrees(self, pwm):
        return pwm / self._factor - self._offset

    def degrees_to_pwm_positions(self, degrees):
        if degrees < 0 or degrees >= 360:
            raise CalibrationError(f'The degree value {degrees} is outside the range 0...<360')
        
        # there could be multiple possible positions, because the tracker might track more than 360 degrees
        

    def calc_calibration(self):
        self._is_calibrated = False

        culminated_factors = 0
        last_degrees = None
        last_pwm = None
        
        sorted_positions = sorted(self._positions)
        for degrees in sorted_positions:
            pwm = self._positions[degrees]
            if not last_degrees == None:
                diff_degrees = degrees - last_degrees
                diff_pwm = pwm - last_pwm
                factor = float(diff_pwm) / float(diff_degrees)
                culminated_factors += factor
            last_degrees = diff_degrees
            last_pwm = pwm

        if (culminated_factors == 0):
            return

        self._factor = culminated_factors / len(self._positions)
        self._offset = sum([sorted_positions[degrees] - degrees * self._factor for degrees in sorted_positions]) / len(self._positions)
        self._min_degrees = self._pwm_to_degrees(self._min_pwm)
        self._max_degrees = self._pwm_to_degrees(self._max_pwm)

        


class TrackerCalibration:
    def __init__(self):
        self._pan_servo_id = 0
        self._tilt_servo_id = 1
        self._pan = AxisCalibration()
        self._tilt = AxisCalibration()

    def is_calibrated(self):
        return self._pan.is_calibrated() and self._tilt.is_calibrated()


class ServoTrackingAntenna(TrackingAntenna):
    def __init__(self, controller):
        self._controller = controller
        

    