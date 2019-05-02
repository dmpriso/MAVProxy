from tracking_antenna import TrackingAntenna
from servo_controller import ServoController

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
        

    