import commands2
from subsystems.swervesubsystem import SwerveSubsystem
from constants import RobotConstants
from subsystems.swervemodule import SwerveModule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import time

class rotateToSpeakerCommand(commands2.Command):
    def __init__(self, swerve: SwerveSubsystem, rotation) -> None:
        super().__init__()
        self.swerve = swerve
        self.rotation = rotation
        

    def initialize(self):
        self.frontLeft.setDesiredState(SwerveModuleState(0, self.rotation))
        self.frontRight.setDesiredState(SwerveModuleState(0, self.rotation))
        self.backLeft.setDesiredState(SwerveModuleState(0, self.rotation))
        self.backRight.setDesiredState(SwerveModuleState(0, self.rotation))

    def execute(self) -> None:
        self.frontLeft.setDesiredState(SwerveModuleState(0.5, self.rotation))
        self.frontRight.setDesiredState(SwerveModuleState(0.5, self.rotation))
        self.backLeft.setDesiredState(SwerveModuleState(0.5, self.rotation))
        self.backRight.setDesiredState(SwerveModuleState(0.5, self.rotation))

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
    
    