import commands2

from subsystems.drivesubsystem import DriveSubsystem
import robotcontainer as container

import math

import constants

class balanceCommand(commands2.CommandBase):
    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__()
        self.drive = drive

    def initialize(self):
        pass

    def execute(self) -> None:
        container.drive.balanceSensitivitySub.get()
        gyroRad = container.drive.gyro.getYaw() * (math.pi / 180)
        pitchAngle = container.drive.gyro.getPitch()
        speed = constants.maxBalanceSpeed * 2 / (1 + math.e ** (-constants.balanceSensitivity * (self.pitchAngle / constants.maxBalanceAngle))) - constants.maxBalanceSpeed  # min(max(-abs(self.pitchAngle) + , 0), 1)

        self.drive.leftTalon.set(self.speed)
        self.drive.rightTalon.set(self.speed)
        self.drive.leftTalon2.set(self.speed)
        self.drive.rightTalon2.set(self.speed)

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
    #Dont want function to terminate unil Auto is over




