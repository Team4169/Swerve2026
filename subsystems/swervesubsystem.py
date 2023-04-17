from .swervemodule import swervemodule
import commands2
import constants
import wpilib
import navx
import threading
import time
import math
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState

from wpimath.geometry import Pose2d


class SwerveSubsystem (commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        self.sd = wpilib.SmartDashboard

        self.frontLeft = swervemodule(constants.frontLeftDrivingMotorID, 
                                constants.frontLeftTurningMotorID, 
                                constants.frontLeftDrivingMotorReversed, 
                                constants.frontLeftTurningMotorReversed, 
                                constants.frontLeftAbsoluteEncoderId, 
                                constants.frontLeftAbsoluteEncoderOffset, 
                                constants.frontLeftAbsoluteEncoderReversed)
        
        self.frontRight = swervemodule(constants.frontRightDrivingMotorID, 
                                constants.frontRightTurningMotorID, 
                                constants.frontRightDrivingMotorReversed, 
                                constants.frontRightTurningMotorReversed, 
                                constants.frontRightAbsoluteEncoderId, 
                                constants.frontRightAbsoluteEncoderOffset, 
                                constants.frontRightAbsoluteEncoderReversed)
        
        self.backLeft = swervemodule(constants.backLeftDrivingMotorID, 
                                constants.backLeftTurningMotorID, 
                                constants.backLeftDrivingMotorReversed, 
                                constants.backLeftTurningMotorReversed, 
                                constants.backLeftAbsoluteEncoderId, 
                                constants.backLeftAbsoluteEncoderOffset, 
                                constants.backLeftAbsoluteEncoderReversed)
        
        self.backRight = swervemodule(constants.backRightDrivingMotorID, 
                                constants.backRightTurningMotorID, 
                                constants.backRightDrivingMotorReversed, 
                                constants.backRightTurningMotorReversed, 
                                constants.backRightAbsoluteEncoderId, 
                                constants.backRightAbsoluteEncoderOffset, 
                                constants.backRightAbsoluteEncoderReversed)
        self.gyro = navx.AHRS(wpilib.SerialPort.Port.kUSB1)
            #the odometry class tracks the robot position over time
            #we can use the gyro in order to determnine the error from our auton path and correct it
        self.odometer = SwerveDrive4Odometry(constants.kDriveKinematics, Rotation2d(0), self.getModuleStatesOld())

        thread = threading.Thread(target=self.zero_heading_after_delay)

        thread.start()

    #~ Gyro Commands
    def zero_heading_after_delay(self):
        try:
            time.sleep(1)
            self.zero_heading()
        except Exception as e:
            pass

    def zeroHeading(self):
        self.gyro.reset()
        
    def getHeading(self):
        return math.remainder(self.gyro.getAngle(), 360)
    
    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPoise(self) -> Pose2d:
        return self.odometer.getPose()
    
    def resetOdometry(self, pose: Pose2d):
        self.odometer.resetPosition(pose, self.getRotation2d())
    
    def stopModuels(self):
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def setModuleStates(self, states: list[SwerveModuleState]):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, constants.kphysicalMaxSpeedMetersPerSecond)
        self.frontLeft.setDesiredState(states[0])
        self.frontRight.setDesiredState(states[1])
        self.backLeft.setDesiredState(states[2])
        self.backRight.setDesiredState(states[3])
    
    def getModuleStates(self) -> tuple[SwerveModulePosition]:
        return (
                self.frontRight.getSwerveModulePosition(),
                self.backRight.getSwerveModulePosition(),
                self.frontLeft.getSwerveModulePosition(),
                self.backLeft.getSwerveModulePosition()
                )
    def getModuleStatesOld(self) -> tuple[SwerveModulePosition]:
        return (
                SwerveModulePosition(self.frontRight.getDrivingPosition(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.backRight.getDrivingPosition(), Rotation2d(self.backRight.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.frontLeft.getDrivingPosition(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.backLeft.getDrivingPosition(), Rotation2d(self.backLeft.getAbsoluteEncoderRad()))
                )
    def periodic(self) -> None:
        self.sd.putNumber("Gyro", self.getHeading())
        self.odometer.update(self.getRotation2d(), 
                            SwerveModulePosition(self.frontRight.getDrivingPosition(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.backRight.getDrivingPosition(), Rotation2d(self.backRight.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.frontLeft.getDrivingPosition(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.backLeft.getDrivingPosition(), Rotation2d(self.backLeft.getAbsoluteEncoderRad()))
                            )
        self.sd.putString("Robot Location", str(self.getPoise()))
        
