from .swervemodule import swervemodule
import commands2
import constants
import wpilib
import navx

# import threading
import time
import math
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveDrive4Kinematics

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
        
    def SwerveSubsytem(self):
        # thread = threading.Thread(target=self.delay)
        # thread.start()
        self.zeroHeading()

    def zeroHeading(self):
        self.gyro.reset()

    def delay(self):
        time.sleep(1)
        self.zeroHeading()
        
    
    def getHeading(self):
        return math.remainder(self.gyro.getAngle(), 360)
    
    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def stopModuels(self):
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def setModuleStates(self, states: list[SwerveModuleState]):
        SwerveDrive4Kinematics.normalizeWheelSpeeds(states, constants.kphysicalMaxSpeedMetersPerSecond)
        self.frontLeft.setDesiredState(states[0])
        self.frontRight.setDesiredState(states[1])
        self.backLeft.setDesiredState(states[2])
        self.backRight.setDesiredState(states[3])
    
    def periodic(self) -> None:
        self.sd.putNumber("Gyro", self.getHeading())
