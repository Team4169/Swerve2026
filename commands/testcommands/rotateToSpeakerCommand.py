import commands2
from subsystems.swervesubsystem import SwerveSubsystem
from constants import RobotConstants
from subsystems.swervemodule import swervemodule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import time

class rotateToSpeakerCommand(commands2.CommandBase):
    def __init__(self, swerve: SwerveSubsystem, rotation) -> None:
        super().__init__()
        self.swerve = swerve
        self.rotation = rotation
        

    def initialize(self):
        self.startRot = Rotation2d(0, 0)
        self.swerve.frontLeft.setDesiredState(SwerveModuleState(0, self.startRot))
        self.swerve.frontRight.setDesiredState(SwerveModuleState(0, self.startRot))
        self.swerve.backLeft.setDesiredState(SwerveModuleState(0, self.startRot))
        self.swerve.backRight.setDesiredState(SwerveModuleState(0, self.startRot))

    def execute(self) -> None:
        #! this might need to be in the end function idk
        self.frontLeft.setDesiredState(SwerveModuleState(0, self.rotation))
        self.frontRight.setDesiredState(SwerveModuleState(0, self.rotation))
        self.backLeft.setDesiredState(SwerveModuleState(0, self.rotation))
        self.backRight.setDesiredState(SwerveModuleState(0, self.rotation))

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
    
    