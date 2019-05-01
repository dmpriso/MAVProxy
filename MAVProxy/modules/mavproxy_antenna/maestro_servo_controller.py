from serial import Serial
from servo_controller import ServoController

def get_value_bytes(value):
    '''Returns value encoded as two bytes with only 7 bits set.'''
    return [value & 0x7F, (value >> 7) & 0x7F]

class MaestroServoController(ServoController):
    def __init__(self, path = '/dev/ttyACM0', deviceNumber = 0x0C):
        if not path == None:
            self._ser = Serial(path)
        else:
            self._ser = None
            print('WARNING Test mode without serial output.')
        self._cmd_prefix = [0xAA, deviceNumber]
        self._last_cmd = []
        

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        if not self._ser == None:
            self._ser.close()
    
    def _send_command(self, bytes):
        '''Sends a command to the maestro, appending prefix and device ID.'''
        send_bytes = self._cmd_prefix + bytes
        packet = bytearray()
        [packet.append(b) for b in (send_bytes)]
        self._last_cmd = send_bytes[:]
        if not self._ser == None:
            self._ser.write(packet)

    def _send_servo_command(self, command_number, channel_number, payload_bytes):
        '''Sends a command to the servo of the given ID'''
        self._send_command([command_number, channel_number] + payload_bytes)
        
    def _send_servo_command_value(self, command_number, channel_number, value):
        '''Sends a command to the given channel, appending the encoded value.'''
        self._send_servo_command(command_number, channel_number, get_value_bytes(int(value)))

    def set_target(self, channel_number, pwm):
        '''Sets the target position of the given channel to the PWM number'''
        self._send_servo_command_value(0x04, channel_number, pwm * 4)

    def set_speed(self, channel_number, us_per_second):
        '''Limits the maximum position change / speed of the given channel.'''
        #  The speed limit is given in units of (0.25 μs)/(10 ms)
        speed = us_per_second * 4.0 / 100.0
        speed = min(max(speed, 1), 16000)
        self._send_servo_command_value(0x07, channel_number, speed)

    def set_acceleration(self, channel_number, us_per_seconds_squared):
        '''Sets the acceleration of position changes'''
        # The acceleration limit is a value from 0 to 255 in units of (0.25 μs)/(10 ms)/(80 ms),
        acc = us_per_seconds_squared * 4.0 / 12.5
        acc = min(max(acc, 1), 255)
        self._send_servo_command_value(0x09, channel_number, acc)

if __name__ == '__main__':
    def make_hex_str(bytes):
        return f'{["0x" + "{:02x}".format(b).upper() for b in bytes]}'

    def test_compare_cmd(expect, cmd):
        '''Compares a command with the expected value for unit testing.'''
        print(f'Expected {make_hex_str(expect)}')
        print(f' Encoded {make_hex_str(cmd)}')
        if not expect == cmd:
            print(f'NO MATCH')
            raise AssertionError()

    with MaestroServoController(None) as ctrl:
        ctrl.set_target(1, 1500)
        test_compare_cmd([0xAA, 0x0C, 0x04, 0x01, 0x70, 0x2E], ctrl._last_cmd)
        ctrl.set_speed(0, 3500)
        test_compare_cmd([0xAA, 0x0C, 0x07, 0x00, 0x0C, 0x01], ctrl._last_cmd)
        ctrl.set_acceleration(2, 12.5)
        test_compare_cmd([0xAA, 0x0C, 0x09, 0x02, 0x04, 0x00], ctrl._last_cmd)
        

