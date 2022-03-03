import commands2

import constants

from .drivedistance import DriveDistance
from .movecommand import MoveCommand
from subsystems.drivesubsystem import DriveSubsystem
from .reset_gyro import ResetGyro

#parallel to the angled line and and inch from the intersection across from the hub
class newPath(commands2.SequentialCommandGroup):
    """
    A complex auto command that drives forward, releases a hatch, and then drives backward.
    """

    def __init__(self, drive: DriveSubsystem):
        super().__init__(
            # Drive forward the specified distance
            ResetGyro(drive),
            MoveCommand(3.75, 0, drive),
            #pick up balls
            MoveCommand(0, 180, drive),
            MoveCommand(9, 180, drive),
            MoveCommand(0, 195, drive),
            MoveCommand(1.5, 195, drive),
            #drop balls
            MoveCommand(-1.5, 195, drive),
            
            )
