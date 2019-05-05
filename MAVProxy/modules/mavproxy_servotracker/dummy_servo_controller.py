from MAVProxy.modules.mavproxy_servotracker.servo_controller import ServoController

class DummyServoController(ServoController):
    def close(self):
        '''Performs cleanup.'''
        print('DummyServoController closed.')
    
    def set_target(self, channel_number, pwm):
        '''Sets the target position of the given channel to the PWM number'''
        print(f'DummyServoController channel {channel_number} target pwm {pwm}')

    def set_speed(self, channel_number, us_per_second):
        '''Limits the maximum position change / speed of the given channel.'''
        print(f'DummyServoController channel {channel_number} speed limit us/s {us_per_second}')

    def set_acceleration(self, channel_number, us_per_seconds_squared):
        '''Sets the acceleration of position changes'''
        print(f'DummyServoController channel {channel_number} acc limit us/ss {us_per_seconds_squared}')