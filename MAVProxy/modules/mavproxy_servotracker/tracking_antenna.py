class TrackingAntenna:
    def is_calibrated(self):
        raise NotImplementedError()

    def set_pan_degrees(self, degrees):
        raise NotImplementedError()

    def set_tilt_degrees(self, degrees):
        raise NotImplementedError()