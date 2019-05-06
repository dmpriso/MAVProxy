from MAVProxy.modules.mavproxy_servotracker.servo_controller import ServoController
import time

class DummyServoController(ServoController):
    def __init__(self, immediate_output):
        self._pwms = {}
        self._last_output = 0
        self._immediate_output = immediate_output

    def close(self):
        '''Performs cleanup.'''
        print('DummyServoController closed.')
        
    def print_position(self):
        formatted = ' '.join([f'Channel[{chnl}]={pwm:9.1f}' for (chnl, pwm) in self._pwms.items()])
        print(f'DummyServoController {formatted}')
        self._last_output = time.time()
    
    def set_target(self, channel_number, pwm):
        '''Sets the target position of the given channel to the PWM number'''
        self._pwms[channel_number] = pwm
        if self._immediate_output == True or (time.time() - self._last_output) > 10:
            self.print_position()

    def set_speed(self, channel_number, us_per_second):
        '''Limits the maximum position change / speed of the given channel.'''
        print(f'DummyServoController channel {channel_number} speed limit us/s {us_per_second}')

    def set_acceleration(self, channel_number, us_per_seconds_squared):
        '''Sets the acceleration of position changes'''
        print(f'DummyServoController channel {channel_number} acc limit us/ss {us_per_seconds_squared}')