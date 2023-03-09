import commands2

import constants

#from subsystems.climbingsubsystem import ClimbingSubsystem
from .moveRotateArmPastLocation import MoveRotateArmPastLocation
from .moveRotateArmToLimitSwitch import MoveRotateArmToLimitSwitch


class RotateArmToBack(commands2.SequentialCommandGroup):
    """
    A complex auto command that drives forward, releases a hatch, and then drives backward.
    """

    def __init__(self, climb: ClimbingSubsystem):
        super().__init__(
            # Drive forward the specified distance
            MoveRotateArmPastLocation(constants.rotateArmCloseToBackTicks, False, constants.rotateArmFastSpeed, climb),
            MoveRotateArmToLimitSwitch(constants.rotateArmSlowSpeed, climb),
            )
