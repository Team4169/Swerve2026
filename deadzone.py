import constants

def addDeadzone(val):
    threshold = constants.deadzone
    if val > 0:
        return (val * (1 + threshold)) - threshold
    elif val < 0:
        return (val * (1 - threshold)) - threshold
    else:
        return 0