import commands2

import constants

from .drivedistance import DriveDistance
from .movecommand import MoveCommand
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem as arm
from .reset_gyro import ResetGyro

#from .SnowVeyerCommands.pickUp import pickUp
#from .SnowVeyerCommands.dropOff import dropOff

'''
1.Holding Cone
2.Extend arm and deposit cone
3.Un Extend
4.Turn around
5.Go to Cone
6.Pick Up cone
7.Turn around, drive to drop off
8.Drop off cone

'''


class Auto2023(commands2.SequentialCommandGroup):
    """
    An auto that drops off cone and grabs another cone
    """

    def __init__(self, drive: DriveSubsystem): #def __init__(self, drive: DriveSubsystem, snowveyor: SnowveyorSubsystem):
        super().__init__(
            # Drive forward the specified distance
            ResetGyro(drive),
            #TODO:Find height of top cone rung and good distance from robot
            arm.setArmtoPoint(3,height,0.5),
            #OpenClaw()
            arm.setExtendingArmPercent(0,0.5),
            arm.setRotatingArmAngle(0,0.5),
            MoveCommand(-5,0,drive),
            MoveCommand(0,180, drive),
            ResetGyro(drive),
            MoveCommand(2,0,drive),
            arm.setArmtoPoint(0.7,0.06,0.3),


            MoveCommand
            # MoveCommand(5,240, drive),

            #MoveCommand(5, 360, drive)
        )
