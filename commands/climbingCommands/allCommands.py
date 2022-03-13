import commands2
from commands.climbingCommands.climbUpDown import climbUpDown
from commands.climbingCommands.rotateArm import rotateArm

import constants
from commands.movecommand import MoveCommand
from subsystems.drivesubsystem import DriveSubsystem

#parallel to the angled line and and inch from the intersection across from the hub
class newPath(commands2.SequentialCommandGroup):
    """
    A complex auto command that drives forward, releases a hatch, and then drives backward.
    """

    def __init__(self, drive: DriveSubsystem):
        super().__init__(
            # Drive forward the specified distance
            climbUpDown(0.2),
            rotateArm(-0.2, """ some number for the rotation of the arm to be totally in"""),
            MoveCommand(1.25, 0, drive),
            climbUpDown(-0.2),
            rotateArm(0.2, """ some number for the rotation of the arm to be attached to be on the bar"""),
            climbUpDown(0.2),


            
            )
