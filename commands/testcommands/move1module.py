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
        

    def initialize(self):
        self.startTime = time.time()
        self.runTime = 5
        self.rotation = Rotation2d(0.0, 1.0)
    def execute(self) -> None:
        self.swerve.frontLeft.setDesiredState(SwerveModuleState(.5, self.rotation))
        self.swerve.sd.putBoolean(f"isAutoRunning", True)

    def end(self, interrupted: bool) -> None:
        # self.rotation = Rotation2d(0, 0)
        # self.swerve.frontRight.setDesiredState(SwerveModuleState(0, self.rotation))
        # self.swerve.sd.putBoolean(f"isAutoRunning", False)
        pass 

    def isFinished(self) -> bool:
        # self.currentTime = time.time()
        # if self.currentTime - self.startTime > self.runTime:
        #     return True

        # return False
        return True