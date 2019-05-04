from tracking_antenna import TrackingAntenna
from servo_controller import ServoController
from axis_calibration import AxisCalibration

class ServoTrackingAxis:
    def __init__(self, controller, servo_id):
        self._controller = controller
        self._servo_id = servo_id
        self._calibration = AxisCalibration()
        self._last_pwm = None

    def set_degrees(self, degrees):
        if _calibration.is_calibrated() == False
            # TODO logging
            return

        positions = self._calibration.degrees_to_pwm_positions(degrees)
        if len(positions) == 0
            # TODO logging
            return

        # if there are multiple matching positions, slew to the nearest
        if self._last_pwm == None
            pwm = positions[0]
        else
            pwm = min(positions, key=lambda x:abs(x-self._last_pwm))

        self._last_pwm = pwm
        self._controller.set_target(self._servo_id, pwm)

    def get_calibration(self):
        return self._calibration

class ServoTrackingAntenna:
    def __init__(self, controller):
        self._controller = controller
        self._pan = ServoTrackingAxis(controller, 0)
        self._tilt = ServoTrackingAxis(controller, 1)

    def is_calibrated(self):
        return self.pan_calibration().is_calibrated() and self.tilt_calibration().is_calibrated()

    def calib_status_descr(self):
        return f'Pan axis calibration status: {self.pan_calibration().calib_status_descr()}\n'\
            f'Tilt axis calibration status: {self.tilt_calibration().calib_status_descr()}'

    def pan_calibration(self):
        return self._pan.get_calibration()

    def tilt_calibration(self):
        return self._tilt.get_calibration()

    def set_pan_servo_id(self, servo_id):
        self._pan = ServoTrackingAxis(self._controller, servo_id)

    def set_tilt_servo_id(self, servo_id):
        self._tilt = ServoTrackingAxis(self._controller, servo_id)

    def set_pan_degrees(self, degrees):
        self._pan.set_degrees(degrees)

    def set_tilt_degrees(self, degrees):
        self._tilt.set_degrees(degrees)
