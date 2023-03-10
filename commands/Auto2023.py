import commands2
import robotcontainer as container
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

    #y is initial y value, a is x coord of dest, b is y cord of dest
    def __init__(self, drive: DriveSubsystem):
        super().__init__(

            ResetGyro(drive),
            #TODO:Find height of top cone rung and good distance from robot
            MoveCommand(-5,0,drive),
            MoveCommand(0,180, drive),
            ResetGyro(drive),
            MoveCommand(0,container.angle,drive),
            MoveCommand(container.driveDistance,0,drive),
            MoveCommand(0,-container.angle,drive),
            #Get on balance
        )
