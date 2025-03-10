import commands2
from subsystems.swervesubsystem import SwerveSubsystem
from constants import RobotConstants
from subsystems.swervemodule import SwerveModule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import time

class MoveInACircle(commands2.CommandBase):
    def __init__(self, swerve: SwerveSubsystem) -> None:
        super().__init__()
        self.swerve = swerve
       
        self.angle = 0
        self.speed = 10

    def initialize(self):
        self.rotation = Rotation2d(1.0, 1.0)
        self.swerve.frontLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.frontRight.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.backLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.backRight.setDesiredState(SwerveModuleState(0.25, self.rotation))

    def execute(self) -> None:
        self.rotation = Rotation2d(self.angle)
        self.swerve.frontLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.frontRight.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.backLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.swerve.backRight.setDesiredState(SwerveModuleState(0.25, self.rotation))

        self.angle += self.speed
 

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False