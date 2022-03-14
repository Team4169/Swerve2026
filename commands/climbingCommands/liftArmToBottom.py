import commands2

import constants

from subsystems.climbingsubsystem import ClimbingSubsystem
from .moveLiftArmPastLocation import MoveLiftArmPastLocation
from .moveLiftArmToLimitSwitch import MoveLiftArmToLimitSwitch


class LiftArmToBottom(commands2.SequentialCommandGroup):
    """
    A complex auto command that drives forward, releases a hatch, and then drives backward.
    """

    def __init__(self, climb: ClimbingSubsystem):
        super().__init__(
            # Drive forward the specified distance
            MoveLiftArmPastLocation(constants.liftArmCloseToBottomTicks, True, -constants.liftArmFastSpeed, climb),
            MoveLiftArmToLimitSwitch(-constants.liftArmSlowSpeed, climb),
            )
