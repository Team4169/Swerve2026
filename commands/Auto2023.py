import commands2

import constants

from .drivedistance import DriveDistance
from .movecommand import MoveCommand
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.snowveyorsubsystem import SnowveyorSubsystem
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
    A complex auto command that drives forward, releases a hatch, and then drives backward.
    """

    def __init__(self, drive: DriveSubsystem): #def __init__(self, drive: DriveSubsystem, snowveyor: SnowveyorSubsystem):
        super().__init__(
            # Drive forward the specified distance
            ResetGyro(drive),
            #Extend(Distance)
            #OpenClaw()
            #Extend(-Distance)
            MoveCommand(-5,0,drive),
            MoveCommand(0,180, drive),
            ResetGyro(drive),
            MoveCommand
            # MoveCommand(5,240, drive),

            #MoveCommand(5, 360, drive)
        )
