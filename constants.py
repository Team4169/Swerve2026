#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean RobotConstants. Don't use this for any other purpose!
#

import math
import wpiutil
import UtilCommands
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry._geometry import Translation2d, Pose2d
from wpimath.trajectory import TrapezoidProfileRadians, TrapezoidProfile

#~ Field Measurements/targets
class FildConstants:
    kFieldWidth = UtilCommands.inchesToMeters(27) #2023 charged up change next year
    kFieldLength = UtilCommands.inchesToMeters(54)

#~ Controller Options
class OIConstants:
    kDriverControllerPort = 0
    kArmControllerPort = 1
    deadzone = .1

#~ robot specifications
class RobotConstants:
        
    kTrackWidth = UtilCommands.inchesToMeters(30) #todo: find the actual track width 
        # ? Distance between the right and left wheels
    kWheelBase = UtilCommands.inchesToMeters(30) #todo: find the actual wheel base
        # ? Distance between the front and back wheels
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kTrackWidth / 2, kWheelBase / 2), #frontleft
        Translation2d(kTrackWidth / 2, -kWheelBase / 2), #frontRight
        Translation2d(-kTrackWidth / 2, kWheelBase / 2), #backLeft
        Translation2d(-kTrackWidth / 2, -kWheelBase / 2), # backRight
        # ? location of each swerve module relative to the center of the robot
    )
    kFrontLeftWheelPosition = Translation2d(kTrackWidth / 2, kWheelBase / 2)
    kFrontRightWheelPosition = Translation2d(kTrackWidth / 2, -kWheelBase / 2)
    kBackLeftWheelPosition = Translation2d(kTrackWidth / 2, -kWheelBase / 2)
    kBackRightWheelPosition = Translation2d(-kTrackWidth / 2, -kWheelBase / 2)


    frontLeftDrivingMotorID = 1
    frontLeftTurningMotorID = 11
    frontLeftDrivingMotorReversed = False
    frontLeftTurningMotorReversed = False
    frontLeftAbsoluteEncoderId = 1
    frontLeftAbsoluteEncoderOffset = 0
    frontLeftAbsoluteEncoderReversed = False

    frontRightDrivingMotorID = 2
    frontRightTurningMotorID = 22
    frontRightDrivingMotorReversed = False
    frontRightTurningMotorReversed = False
    frontRightAbsoluteEncoderId = 2
    frontRightAbsoluteEncoderOffset = 0
    frontRightAbsoluteEncoderReversed = False

    backLeftDrivingMotorID = 3
    backLeftTurningMotorID = 33
    backLeftDrivingMotorReversed = False
    backLeftTurningMotorReversed = False
    backLeftAbsoluteEncoderId = 3
    backLeftAbsoluteEncoderOffset = 0
    backLeftAbsoluteEncoderReversed = False

    backRightDrivingMotorID = 4
    backRightTurningMotorID = 44
    backRightDrivingMotorReversed = False
    backRightTurningMotorReversed = False
    backRightAbsoluteEncoderId = 4
    backRightAbsoluteEncoderOffset = 0
    backRightAbsoluteEncoderReversed = False

    kPhysicalMaxAngularSpeedRadiansPerSecond  = 2 * 2 * math.pi
    kphysicalMaxSpeedMetersPerSecond = 12 #^ not sure how to get this, maybe look online and multiply by gear ratio?

    kTeleopDriveMaxAccelerationUnitsPerSec = 1 # ? 0 to 1, represents move restriciton
    kTeleopDriveMaxAngularAccelerationUnitsPerSec = 1

    kTeleopDriveMaxSpeedMetersPerSecond = 12 
    kTeleopDriveMaxAngularSpeedRadiansPerSecond = 12

    testDrivingMotorID = 13
    testTurningMotorID = 14
# ~ Swerve Constants 


#todo Change above
class ModuleConstants:
    kWheelDiameterMeters = UtilCommands.inchesToMeters(4) #^ wheele listed as "Wheel, Billet, 4"OD x 1.5"W (MK4/4i)"" I think that's what the 4 OD means

    kDriveMotorGearRatio = 1/5.43 #^placeholder
    kTurningMotorGearRatio = 1/5.43 #^placeholder

    kDrivingEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    KDrivingEncoderRPM2MeterPerSec = kDrivingEncoderRot2Meter / 60

    kWheelDistancePerRadian = kDrivingEncoderRot2Meter / (2 * math.pi)

    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60 

    kPTurning = .5 #? turning PID controller per wheele


# ~ Auto Constants

class AutoConstants:
        kMaxSpeedMetersPerSecond = .75
        kMaxAccelerationMetersPerSecondSquared = .75
        kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4

        kPXController = 1.0
        kPYController = 1.0

        kMaxAngularSpeedRadiansPerSecond = RobotConstants.kphysicalMaxSpeedMetersPerSecond / 10

        kPThetaController = 1.0
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared,
        )
class sim:
    kSimTargetName = "SimTarget"
    # kSimDefaultTargetLocation = Pose2d(
    #     kFieldLength / 2, kFieldWidth / 2, 180 * kRadiansPerDegree
    # )
    """[meters, meters, radians]"""

    kSimDefaultRobotLocation = Pose2d(FildConstants.kFieldLength / 2, FildConstants.kFieldWidth / 2, 0)
    # kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in
    # kSimBallName = "SimBall"
    # kSimDefaultBallLocation = Pose2d(kFieldLength / 4, kFieldWidth / 2, 0)

    """meters"""

    kSimRobotPoseArrayKey = "SimRobotPoseArray"
    kSimTargetPoseArrayKey = "SimTargetPoseArray"
    kSimBallPoseArrayKey = "SimBallPoseArray"
    kSimTargetHeightKey = "SimTargetHeight"
    kSimTargetTrackingModuleName = "sim_target_tracker"
    kSimTargetUpperHubRadius = 2

    kSimFrontLeftDriveMotorPort = 0
    kSimFrontLeftSteerMotorPort = 1
    kSimFrontRightDriveMotorPort = 2
    kSimFrontRightSteerMotorPort = 3
    kSimBackLeftDriveMotorPort = 4
    kSimBackLeftSteerMotorPort = 5
    kSimBackRightDriveMotorPort = 6
    kSimBackRightSteerMotorPort = 7


    kSimFrontLeftDriveEncoderPorts = (16, 1)
    kSimFrontLeftSteerEncoderPorts = (2, 3)
    kSimFrontRightDriveEncoderPorts = (4, 5)
    kSimFrontRightSteerEncoderPorts = (6, 7)
    kSimBackLeftDriveEncoderPorts = (8, 9)
    kSimBackLeftSteerEncoderPorts = (10, 11)
    kSimBackRightDriveEncoderPorts = (12, 13)
    kSimBackRightSteerEncoderPorts = (14, 15)
