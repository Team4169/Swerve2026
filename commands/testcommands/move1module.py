import commands2
from subsystems.swervesubsystem import SwerveSubsystem
from constants import RobotConstants
from subsystems.swervemodule import swervemodule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import time

class move1module(commands2.CommandBase):
    def __init__(self, swerve: SwerveSubsystem) -> None:
        super().__init__()
        self.swerve = swerve
        self.startTime = time.time()
        self.runTime = 5

    def initialize(self):
        self.rotation = Rotation2d(1.0, 1.0)
        self.swerve.frontLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))

    def execute(self) -> None:
       pass 

    def end(self, interrupted: bool) -> None:
        self.rotation = Rotation2d(0, 0)
        self.swerve.frontLeft.setDesiredState(SwerveModuleState(0, self.rotation))

    def isFinished(self) -> bool:
        self.currentTime = time.time()
        if self.currentTime - self.startTime > self.runTime:
            return True

        return False