def sign(x):
    return (x > 0) - (x < 0)

def avg(list):
    return sum(list) / len(list)

def angle_diff(fromAngle, toAngle):
    '''Calculates the angle between fromAngle and toAngle, taking rotation (wrap-around after 360) into account.'''
    diff = float(toAngle) - float(fromAngle)
    if abs(diff) <= 180.0:
        return diff
    # let's assume 350 ... 10
    return angle_diff(fromAngle + 360.0 * float(sign(diff)), toAngle)

def normalize_angle(angleDegrees):
    '''Makes sure the given angle in the range from 0 to <360.'''
    return float(angleDegrees) % 360.0
    