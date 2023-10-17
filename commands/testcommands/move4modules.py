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
        # self.frontLeft = swervemodule(RobotConstants.frontLeftDrivingMotorID, 
        #                         RobotConstants.frontLeftTurningMotorID, 
        #                         RobotConstants.frontLeftDrivingMotorReversed, 
        #                         RobotConstants.frontLeftTurningMotorReversed, 
        #                         RobotConstants.frontLeftAbsoluteEncoderId, 
        #                         RobotConstants.frontLeftAbsoluteEncoderOffset, 
        #                         RobotConstants.frontLeftAbsoluteEncoderReversed)
        
        self.frontRight = swervemodule(RobotConstants.frontRightDrivingMotorID, 
                                RobotConstants.frontRightTurningMotorID, 
                                RobotConstants.frontRightDrivingMotorReversed, 
                                RobotConstants.frontRightTurningMotorReversed, 
                                RobotConstants.frontRightAbsoluteEncoderId, 
                                RobotConstants.frontRightAbsoluteEncoderOffset, 
                                RobotConstants.frontRightAbsoluteEncoderReversed)
        
        self.backLeft = swervemodule(RobotConstants.backLeftDrivingMotorID, 
                                RobotConstants.backLeftTurningMotorID, 
                                RobotConstants.backLeftDrivingMotorReversed, 
                                RobotConstants.backLeftTurningMotorReversed, 
                                RobotConstants.backLeftAbsoluteEncoderId, 
                                RobotConstants.backLeftAbsoluteEncoderOffset, 
                                RobotConstants.backLeftAbsoluteEncoderReversed)
        
        self.backRight = swervemodule(RobotConstants.backRightDrivingMotorID, 
                                RobotConstants.backRightTurningMotorID, 
                                RobotConstants.backRightDrivingMotorReversed, 
                                RobotConstants.backRightTurningMotorReversed, 
                                RobotConstants.backRightAbsoluteEncoderId, 
                                RobotConstants.backRightAbsoluteEncoderOffset, 
                                RobotConstants.backRightAbsoluteEncoderReversed)
        
        self.startTime = time.time()
        self.runTime = 5


    def initialize(self):
        self.rotation = Rotation2d(1.0, 1.0)
        self.frontLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.frontRight.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.backLeft.setDesiredState(SwerveModuleState(0.25, self.rotation))
        self.backRight.setDesiredState(SwerveModuleState(0.25, self.rotation))

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