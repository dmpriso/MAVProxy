#!/usr/bin/env python
'''
antenna pointing module
Daniel Modler
May 2019
'''

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_servotracker.maestro_servo_controller import MaestroServoController
from MAVProxy.modules.mavproxy_servotracker.dummy_servo_controller import DummyServoController
from MAVProxy.modules.mavproxy_servotracker.servo_tracking_antenna import ServoTrackingAntenna
from MAVProxy.modules.mavproxy_servotracker.calc_tools import normalize_angle

def servoid_in_range(id):
    return id >= 0 and id <= 5

def pwm_in_range(pwm):
    return pwm >= 700 and pwm <= 2300

def direction_by_name(name):
    directions = {
        'north': 0,
        'east': 90,
        'south': 180,
        'west': 270,
        'horizontal': 0,
        'vertical': 90
    }
    direction = directions.get(name)
    if direction is None and str(name).isdigit():
        direction = int(name)
        return None if direction < 0 or direction >= 360 else direction
    return direction

class ServoTrackerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ServoTrackerModule, self).__init__(mpstate, "servotracker", "servo tracker module", public=True)
        self.add_command('servotracker', self.cmd_servotracker, 'servo tracker test & calibration')
        self.controller = None
        self.tracking_antenna = None
        self.tracker_info = None
        self.output_channels = {}

    def print_usage(self):
        print('Usage:')
        print('servotracker <help|status|init|pwm|slew|calib|tune>')
        print('')
        print('Sub-commands:')
        print('')
        print('help         Displays this help text')
        print('')
        print('status       Show tracker and calibration status')
        print('')
        print('init         Initializes the tracker. Syntax:')
        print('             init <maestro|dummy> PAN_SERVO_CHANNEL TILT_SERVO_CHANNEL [PORT]')
        print('             Initializes a pololu maestro or a dummy servo controller on the specified port (maestro only)')
        print('')
        print('             Example:')
        print('             servotracker init maestro 0 1 /dev/ttyACM0')
        print('             (Initializes with pan servo attached at channel 0 and tilt servo at channel 1)')
        print('')
        print('pwm          Slews a servo to the given servo position. Syntax:')
        print('             pwm <pan|tilt> PWM_VALUE')
        print('')
        print('             Example:')
        print('             servotracker pwm pan 1500')
        print('             (Slews the pan servo to center position)')
        print('')
        print('slew         Slews an axis to the given degree value. Syntax:')
        print('             slew <pan|tilt> DEGREES')
        print('')
        print('             Example:')
        print('             servotracker slew tilt 80')
        print('             (Slews the tilt axis to a high elevation. Tracker must be calibrated!)')
        print('')
        print('calib        Adds the current servo position as calibration point. Syntax:')
        print('             calib pan <north|east|south|west|min|max|clear|details|DEGREES> [PWM]')
        print('             calib tilt <horizontal|vertical|min|max|clear|details|DEGREES> [PWM]')
        print('')
        print('             Example:')
        print('             servotracker pwm pan 1200')
        print('             servotracker calib pan current north')
        print('             (Learns the pwm value 1200 as north calibration)')
        print('')
        print('             servotracker calib tilt 90 1800')
        print('             (Learns the pwm value 1800 as zenith elevation)')
        print('')
        print('             servotracker calib pan min 1000')
        print('             (Learns the pwm value 1000 as lowest possible pwm value)')
        print('')
        print('details      Returns details of the axis calibration. Useful for debugging purposes.')
        print('tune         Tracker speed/acceleration tuning. Syntax:')
        print('             tune <pan|tilt> <speed|acceleration> VALUE')
        print('')
        print('             Example:')
        print('             servotracker tune speed 50')
        print('             (Limits the speed to a PWM change of 50us per second)')
        print('')
        print('             servotracker tune acc 15')
        print('             (Limits the acceleration to a PWM change of 50us per second squared)')

    def print_status(self):
        if self.tracking_antenna is None:
            print('Tracker not initialized.')
            return

        print('Tracker: ' + self.tracker_info)

        print(f'Tracker {"NOT" if self.tracking_antenna.is_calibrated() == False else ""} calibrated')

        print(self.tracking_antenna.calib_status_descr())

    def subsubcmd_init(self, tracker_type, args):
        if not (len(args) == 2 or len(args) == 3):
            return False

        pan_servo_id = int(args[0])
        tilt_servo_id = int(args[1])
        tty_iface = None if len(args) <= 2 else args[2]

        if (
                pan_servo_id == tilt_servo_id or 
                servoid_in_range(pan_servo_id) == False or 
                servoid_in_range(tilt_servo_id) == False
           ):
            print('Invalid servo ID')
            return False

        self.output_channels = {
            'pan': pan_servo_id,
            'tilt': tilt_servo_id
        }

        if tracker_type == 'maestro':
            self.controller = MaestroServoController(tty_iface)
        elif tracker_type == 'dummy':
            self.controller = DummyServoController(tty_iface == 'immediate')
        else:
            print('Unknown tracker type. Supported tracker types are "maestro", "dummy"')
            return True

        self.tracking_antenna = ServoTrackingAntenna(self.controller)
        self.tracker_info = f'Servo controller type={tracker_type}. Pan servo channel={pan_servo_id}, tilt servo channel={tilt_servo_id}'
        if not tty_iface == None:
            self.tracker_info = self.tracker_info + f'. Serial interface={tty_iface}'

        print('Initialized: ' + self.tracker_info)

        return True

    def get_axis(self, axis):
        if axis == 'pan':
            return self.tracking_antenna.pan_axis()
        elif axis == 'tilt':
            return self.tracking_antenna.tilt_axis()
        return None

    def subsubcmd_pwm(self, axis, pwm):
        if self.tracking_antenna is None:
            print('No tracker configured. Run servotracker init first.')
            return True

        axis = self.get_axis(axis)
        if axis is None:
            return False
        pwm = int(pwm)
        if pwm_in_range(pwm) == False:
            return False
        axis.set_pwm(pwm)

        return True

    def subsubcmd_slew(self, axis, degrees):
        if self.tracking_antenna is None:
            print('No tracker configured. Run servotracker init first.')
            return True
        elif self.tracking_antenna.is_calibrated() == False:
            print('Tracker not calibrated, run calibration first')
            return True

        if axis == 'pan':
            self._slew_pan_to(degrees)
        elif axis == 'tilt':
            self._slew_tilt_to(degrees)
        else:
            return False

        return True

    def subsubcmd_calib(self, axis_name, direction, pwm):
        if self.tracking_antenna is None:
            print('No tracker configured. Run servotracker init first.')
            return True

        axis = self.get_axis(axis_name)
        if axis is None:
            return False

        if (pwm is None):
            pwm = axis.get_pwm()
        else:
            pwm = int(pwm)
            if pwm_in_range(pwm) == False:
                print('Invalid pwm value')
                return True

        calib = axis.get_calibration()

        if direction == 'clear':
            calib.clear()
            print(f'Calibration for {axis_name} cleared')
            return True
        elif direction == 'max':
            calib.set_max_pwm(pwm)
            print(f'Max PWM value for {axis_name} set to {pwm}')
        elif direction == 'min':
            calib.set_min_pwm(pwm)
            print(f'Min PWM value for {axis_name} set to {pwm}')
        elif direction == 'details':
            print(calib.get_calibration_details())
            return True
        else:
            degrees = direction_by_name(direction)
            if degrees is None:
                return False
            calib.add_calibration_point(degrees, pwm)
            print(f'Calibration point added: pwm={pwm} degrees={degrees}')

        print(f'Calculating ...')
        calib.calc_calibration()
        print(f'{axis_name.capitalize()} axis {"NOT " if calib.is_calibrated() == False else ""}CALIBRATED')
        if (calib.is_calibrated()):
            warnings = calib.get_calibration_warnings()
            if warnings:
                print(warnings)

        return True

    def subsubcmd_tune(self, axis, tune, value):
        if self.tracking_antenna is None:
            print('No tracker configured. Run servotracker init first.')
            return True        

        channel = self.output_channels.get(axis)
        if channel is None:
            return False

        value = float(value)
        if value < 0 or value > 1000:
            print('Value must be > 0 and <= 1000')
            return True

        if tune == 'speed':
            self.controller.set_speed(channel, value)
        elif tune == 'acceleration':
            self.controller.set_acceleration(channel, value)
        else:
            return False

        return True

    def subcmd_servotracker(self, cmd, args):
        if cmd == 'status':
            self.print_status()
            return
        elif cmd == 'help':
            self.print_usage()
            return
        elif cmd == 'init' and len(args) >= 1:
            if self.subsubcmd_init(args[0], args[1:]) == True:
                return
        elif cmd == 'pwm' and len(args) == 2:
            if self.subsubcmd_pwm(args[0], args[1]) == True:
                return
        elif cmd == 'slew' and len(args) == 2:
            if self.subsubcmd_slew(args[0], args[1]) == True:
                return
        elif cmd == 'calib' and len(args) in [2, 3]:
            if self.subsubcmd_calib(args[0], args[1], None if len(args) == 2 else args[2]) == True:
                return            
        elif cmd == 'tune' and len(args) == 3:
            if self.subsubcmd_tune(args[0], args[1], args[2]) == True:
                return
        
        print('Invalid command or syntax. Type servotracker help for usage information')

    def cmd_servotracker(self, args):
        '''servo tracker test and calibration'''
        if len(args) == 0:
            self.print_usage()
        elif len(args) >= 1:
            self.subcmd_servotracker(args[0], args[1:])

    def _slew_pan_to(self, pan):
        self.tracking_antenna.set_pan_degrees(normalize_angle(pan))

    def _slew_tilt_to(self, tilt):
        # we limit tilt to useful numbers
        tilt = normalize_angle(tilt)
        if tilt > 180:
            tilt = max(tilt, 315.0) # this equals -45deg
        else:
            tilt = min(tilt, 90.0)
        self.tracking_antenna.set_tilt_degrees(tilt)

    def slew_to(self, pan, tilt):
        if self.tracking_antenna is None or self.tracking_antenna.is_calibrated() == False:
            return False

        '''Programmable slew for other modules'''
        self._slew_pan_to(pan)
        self._slew_tilt_to(tilt)

        return True
        

def init(mpstate):
    '''initialise module'''
    return ServoTrackerModule(mpstate)
