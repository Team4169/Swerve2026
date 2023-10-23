import commands2
from subsystems.swervesubsystem import SwerveSubsystem
from constants import RobotConstants
from subsystems.swervemodule import swervemodule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import time

class move4modules(commands2.CommandBase):
    def __init__(self, swerve: SwerveSubsystem) -> None:
        super().__init__()
        self.swerve = swerve
        self.startTime = time.time()
        self.runTime = 5


    def initialize(self):
        self.rotation = Rotation2d(1.0, 1.0)
        self.swerve.frontLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.frontRight.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.backLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.backRight.setDesiredState(SwerveModuleState(0.25, self.rotation))

    def execute(self) -> None:
        self.sd.putNumber("Module State", self.frontLeft.getState())
 

    def end(self, interrupted: bool) -> None:
        self.rotation = Rotation2d(0, 0)
        self.frontLeft.setDesiredState(SwerveModuleState(0, self.rotation))
        self.frontRight.setDesiredState(SwerveModuleState(0, self.rotation))
        self.backLeft.setDesiredState(SwerveModuleState(0, self.rotation))
        self.backRight.setDesiredState(SwerveModuleState(0, self.rotation))

    def isFinished(self) -> bool:
        self.currentTime = time.time()
        if self.currentTime - self.startingTime > self.runTime:
            return True

        return False