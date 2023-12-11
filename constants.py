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
        
    kTrackWidth = UtilCommands.inchesToMeters(20) #found with measuring tape
        # ? Distance between the right and left wheels
    kWheelBase = UtilCommands.inchesToMeters(20) #todo: find the actual wheel base
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
    frontLeftAbsoluteEncoderId = 1 #DIO port ID
    frontLeftAbsoluteEncoderOffset = 0
    frontLeftAbsoluteEncoderReversed = False

    frontRightDrivingMotorID = 2
    frontRightTurningMotorID = 22
    frontRightDrivingMotorReversed = False
    frontRightTurningMotorReversed = False
    frontRightAbsoluteEncoderId = 2
    frontRightAbsoluteEncoderOffset = 0
    frontRightAbsoluteEncoderReversed = False

    backRightDrivingMotorID = 3
    backRightTurningMotorID = 33
    backRightDrivingMotorReversed = False
    backRightTurningMotorReversed = False
    backRightAbsoluteEncoderId = 3
    backRightAbsoluteEncoderOffset = 0
    backRightAbsoluteEncoderReversed = False
    
    backLeftDrivingMotorID = 4
    backLeftTurningMotorID = 44
    backLeftDrivingMotorReversed = False
    backLeftTurningMotorReversed = False
    backLeftAbsoluteEncoderId = 4
    backLeftAbsoluteEncoderOffset = 0
    backLeftAbsoluteEncoderReversed = False

    

    kPhysicalMaxAngularSpeedRadiansPerSecond  = 2 * 2 * math.pi
    kphysicalMaxSpeedMetersPerSecond = 1 
    #! This value seems to be the max swerve speed in MPS,
    #! it would serve us to either calculate the max swerve velocity at 100% or take it from the 
    #! WCP page https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options/custom-gear-ratios#possible-gear-ratios-non-flipped
    #^ the above constant is needed to normalize wheel speeds in the case that a speed value above the max is 
    #^ trying to be passed into the setmodulestates function, this will likely not be done manually
    #^ but occur in code at some point

    kTeleopDriveMaxAccelerationUnitsPerSec = 1 # ? 0 to 1, represents move restriciton
    kTeleopDriveMaxAngularAccelerationUnitsPerSec = 1

    kTeleopDriveMaxSpeedMetersPerSecond = 12 
    kTeleopDriveMaxAngularSpeedRadiansPerSecond = 12

    

# ~ Swerve Constants 


#todo Change above
class ModuleConstants:
    kWheelDiameterMeters = UtilCommands.inchesToMeters(4) #^ wheele listed as "Wheel, Billet, 4"OD x 1.5"W (MK4/4i)"" I think that's what the 4 OD means

    kDriveMotorGearRatio = 1/6.75 #^placeholder
    kTurningMotorGearRatio = 1/13 #^13 rotations of the motor = 1 turn of the wheel

    kDrivingEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    KDrivingEncoderRPM2MeterPerSec = kDrivingEncoderRot2Meter / 60

    kWheelDistancePerRadian = kDrivingEncoderRot2Meter / (2 * math.pi)

    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60 

    kPTurning = .5 #? turning PID controller per wheel


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
