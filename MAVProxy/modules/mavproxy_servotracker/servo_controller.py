class ServoController:
    def close(self):
        '''Performs cleanup.'''
        raise NotImplementedError()
    
    def set_target(self, channel_number, pwm):
        '''Sets the target position of the given channel to the PWM number'''
        raise NotImplementedError()

    def set_speed(self, channel_number, us_per_second):
        '''Limits the maximum position change / speed of the given channel.'''
        raise NotImplementedError()

    def set_acceleration(self, channel_number, us_per_seconds_squared):
        '''Sets the acceleration of position changes'''
        raise NotImplementedError()