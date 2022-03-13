import constants

def addDeadzone(val):
    threshold = constants.deadzone
    return (val * (1 + threshold)) - threshold
