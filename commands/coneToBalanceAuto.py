import commands2
import robotcontainer as container
import constants
from .armCommands import *
from .movecommand import MoveCommand
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem as arm
from .reset_gyro import ResetGyro


'''
1.Holding Cone
2.Extend arm and deposit cone
3.Un Extend
4.Turn around
5.Drive to Balance
6.Face streight
7.Get up and balance 

'''


class coneToBalanceAuto(commands2.SequentialCommandGroup):
    """
    An auto that drops off cone and goes onto balance
    """

    def __init__(self, drive: DriveSubsystem):
        super().__init__(
            ResetGyro(drive),
            dropOff(constants.dropOffDistance,constants.coneTargetHeights[container.target]),
            setExtendingArm(0,arm),
            setRotatingArm(0,arm),
            MoveCommand(-5,0,drive),
            MoveCommand(0,180, drive),
            ResetGyro(drive),
            MoveCommand(0,container.getAngle(True),drive),
            MoveCommand(container.getDistance(True),0,drive),
            MoveCommand(0,-container.getAngle(True),drive),
            MoveCommand(0.5,0,drive),
            balanceCommand(drive)
        )
