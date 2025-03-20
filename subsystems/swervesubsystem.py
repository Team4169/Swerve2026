import math, time, threading
import wpilib, commands2, navx

from .swervemodule import SwerveModule
from constants import RobotConstants, AutoConstants
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState
from wpimath.geometry import Pose2d

from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation

import ntcore

from wpimath.kinematics import ChassisSpeeds
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d

class SwerveSubsystem (commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        self.sd = wpilib.SmartDashboard
        
        self.frontLeft = SwerveModule(RobotConstants.frontLeftDrivingMotorID, 
                                RobotConstants.frontLeftTurningMotorID, 
                                RobotConstants.frontLeftDrivingMotorReversed, 
                                RobotConstants.frontLeftTurningMotorReversed, 
                                RobotConstants.frontLeftAbsoluteEncoderId, 
                                RobotConstants.frontLeftAbsoluteEncoderOffset, 
                                RobotConstants.frontLeftAbsoluteEncoderReversed)
        
        self.frontRight = SwerveModule(RobotConstants.frontRightDrivingMotorID, 
                                RobotConstants.frontRightTurningMotorID, 
                                RobotConstants.frontRightDrivingMotorReversed, 
                                RobotConstants.frontRightTurningMotorReversed, 
                                RobotConstants.frontRightAbsoluteEncoderId, 
                                RobotConstants.frontRightAbsoluteEncoderOffset, 
                                RobotConstants.frontRightAbsoluteEncoderReversed)
        
        self.backLeft = SwerveModule(RobotConstants.backLeftDrivingMotorID, 
                                RobotConstants.backLeftTurningMotorID, 
                                RobotConstants.backLeftDrivingMotorReversed, 
                                RobotConstants.backLeftTurningMotorReversed, 
                                RobotConstants.backLeftAbsoluteEncoderId, 
                                RobotConstants.backLeftAbsoluteEncoderOffset, 
                                RobotConstants.backLeftAbsoluteEncoderReversed)
        
        self.backRight = SwerveModule(RobotConstants.backRightDrivingMotorID, 
                                RobotConstants.backRightTurningMotorID, 
                                RobotConstants.backRightDrivingMotorReversed, 
                                RobotConstants.backRightTurningMotorReversed, 
                                RobotConstants.backRightAbsoluteEncoderId, 
                                RobotConstants.backRightAbsoluteEncoderOffset, 
                                RobotConstants.backRightAbsoluteEncoderReversed)


        #SPI for fast communiation, calibrate gyro based on mount direction . calibration docs: https://docs.yagsl.com/devices/gyroscope/navx
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        # self.gyro.BoardAxis(1)

        #the odometry class tracks the robot position over time
        #we can use the gyro in order to determnine the error from our auton path and correct it
        self.odometer = SwerveDrive4Odometry(RobotConstants.kDriveKinematics, Rotation2d(0), self.getModulePositions())

        thread = threading.Thread(target=self.zero_heading_after_delay)

        thread.start()

        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.resetOdometry, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getChassisSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveChassisSpeeds, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
            ),
            config,
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self
        )
        
    #~ Gyro Commands
    def zeroHeading(self):
        self.gyro.reset()
        

    #^used when we are able to adjust gyro with apriltags
    def setHeading(self, angle):
        self.gyro.reset()
        self.gyro.setAngleAdjustment(angle)
        

    def zero_heading_after_delay(self):
        try:
            time.sleep(1)
            self.gyro.reset()
        except Exception as e:
            pass
        
    def getHeading(self):
        return  math.remainder(self.gyro.getAngle(), 360)
        
    
    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self) -> Pose2d:
        return self.odometer.getPose()
    
    def resetOdometry(self, pose: Pose2d):
        self.odometer.resetPosition(
        self.getRotation2d(),
        (
            self.frontLeft.getSwerveModulePosition(),
            self.frontRight.getSwerveModulePosition(),
            self.backLeft.getSwerveModulePosition(),
            self.backRight.getSwerveModulePosition()
        ),
        pose,
        )
    
    def stopModules(self):
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def setModuleStates(self, states: list[SwerveModuleState]):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(tuple(states), RobotConstants.kphysicalMaxSpeedMetersPerSecond)
        self.frontLeft.setDesiredState(states[0])
        self.frontRight.setDesiredState(states[1])
        self.backLeft.setDesiredState(states[2])
        self.backRight.setDesiredState(states[3])

    # def getModuleStates(self) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
    #     return (
    #             self.frontLeft.getSwerveModulePosition(),
    #             self.frontRight.getSwerveModulePosition(),
    #             self.backLeft.getSwerveModulePosition(),
    #             self.backRight.getSwerveModulePosition()
    #             )

    def getChassisSpeeds(self):
        return RobotConstants.kDriveKinematics.toChassisSpeeds(self.getModuleStates())

    def getModulePositions(self) -> tuple[SwerveModulePosition, SwerveModulePosition,SwerveModulePosition,SwerveModulePosition]:
        return (
                SwerveModulePosition(self.frontLeft.getDrivingPosition(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.frontRight.getDrivingPosition(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.backLeft.getDrivingPosition(), Rotation2d(self.backLeft.getAbsoluteEncoderRad())),
                SwerveModulePosition(self.backRight.getDrivingPosition(), Rotation2d(self.backRight.getAbsoluteEncoderRad()))
                )

    def getModuleStates(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return (
            SwerveModuleState(self.frontLeft.getDrivingVelocity(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                SwerveModuleState(self.frontRight.getDrivingVelocity(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                SwerveModuleState(self.backLeft.getDrivingVelocity(), Rotation2d(self.backLeft.getAbsoluteEncoderRad())),
                SwerveModuleState(self.backRight.getDrivingVelocity(), Rotation2d(self.backRight.getAbsoluteEncoderRad()))
        )

    def driveChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, feedForwards): # Check to make sure "feedForwards" should be here
            self.setModuleStates(
            RobotConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        )

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed  #! or kBlue idk this needs testing
    ####return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    # def getAlliance(self):
    #     return True
    #     #~use FMS to find the allinace side

    def periodic(self) -> None:
        self.sd.putNumber("Gyro", self.getHeading())
        self.odometer.update(
                            self.getRotation2d(), 
                            (
                            (SwerveModulePosition(self.frontLeft.getDrivingPosition(), Rotation2d(self.frontLeft.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.frontRight.getDrivingPosition(), Rotation2d(self.frontRight.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.backLeft.getDrivingPosition(), Rotation2d(self.backLeft.getAbsoluteEncoderRad())),
                            SwerveModulePosition(self.backRight.getDrivingPosition(), Rotation2d(self.backRight.getAbsoluteEncoderRad())))
                            )
                            )
        
        # pass
        #~ Camera Pose Updating
        # self.robotR.get()
        # self.robotY.get()
        # self.robotX.get()
        # self.weight.get()

    def lockWheels(self):
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d(1, -1)))
        self.frontRight.setDesiredState(SwerveModuleState(0, Rotation2d(1, 1)))
        self.backLeft.setDesiredState(SwerveModuleState(0, Rotation2d(1, 1)))
        self.backRight.setDesiredState(SwerveModuleState(0, Rotation2d(1, -1)))

        self.frontLeft.setBrakeMode()
        self.frontRight.setBrakeMode()
        self.backLeft.setBrakeMode()
        self.backRight.setBrakeMode()

    def unlockWheels(self):
        self.frontLeft.setCoastMode()
        self.frontRight.setCoastMode()
        self.backLeft.setCoastMode()
        self.backRight.setCoastMode()