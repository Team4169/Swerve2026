#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean RobotConstants. Don't use this for any other purpose!
#

import math, UtilCommands
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry._geometry import Translation2d
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.filter import SlewRateLimiter
from pathplannerlib.path import PathConstraints

#~ Field Measurements/targets
class FildConstants:
    kFieldWidth = UtilCommands.inchesToMeters(27) #2023 charged up change next year
    kFieldLength = UtilCommands.inchesToMeters(54)

#~ Controller Options
class OIConstants:
    kDriverControllerPort = 0
    kArmControllerPort = 1
    deadzone = 0.2
#~ driving constants
class DrivingConstants:
    drivingSpeedLimiter = 1
    rotationSpeedLimiter = 1
    voltageLimiter = SlewRateLimiter(0.5)

#~ robot specification
class RobotConstants:
    kWheelDiameterMeters = UtilCommands.inchesToMeters(4) #^ wheele listed as "Wheel, Billet, 4"OD x 1.5"W (MK4/4i)"" I think that's what the 4 OD means

    kTrackWidth = UtilCommands.inchesToMeters(22) #found with measuring tape
        # ? Distance between the right and left wheels
    kWheelBase = UtilCommands.inchesToMeters(17.5) 
        # ? Distance between the front and back wheels
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(-kTrackWidth / 2, -kWheelBase / 2), #Front Left       
        Translation2d(kTrackWidth / 2, -kWheelBase / 2), #Front Right
        Translation2d(-kTrackWidth / 2, kWheelBase / 2), #Back Left
        Translation2d(kTrackWidth / 2, kWheelBase / 2), #Back Right

        # ? location of each swerve module relative to the center of the robot
    )
    kFrontLeftWheelPosition = Translation2d(kTrackWidth / 2, kWheelBase / 2)
    kFrontRightWheelPosition = Translation2d(kTrackWidth / 2, -kWheelBase / 2)
    kBackLeftWheelPosition = Translation2d(kTrackWidth / 2, -kWheelBase / 2)
    kBackRightWheelPosition = Translation2d(-kTrackWidth / 2, -kWheelBase / 2)

    frontLeftDrivingMotorID = 8
    frontLeftTurningMotorID = 9
    frontLeftDrivingMotorReversed = True
    frontLeftTurningMotorReversed = False
    frontLeftAbsoluteEncoderId = 14 #Front Left #DIO port ID
    frontLeftAbsoluteEncoderOffset = -(math.pi/2 - 1.628) + math.pi # (.5 * math.pi) -.728 + (0.25*math.pi) #0.505#  This should be 1.628
    frontLeftAbsoluteEncoderReversed = True

    frontRightDrivingMotorID = 4
    frontRightTurningMotorID = 6
    frontRightDrivingMotorReversed = False
    frontRightTurningMotorReversed = False
    frontRightAbsoluteEncoderId = 12 #Front Right
    frontRightAbsoluteEncoderOffset = math.pi/6 - 0.01587 #(0.421-0.5*math.pi) - math.pi/6 # 0.770 + (0.5*math.pi)# = 2.340
    frontRightAbsoluteEncoderReversed = True

    backRightDrivingMotorID = 2
    backRightTurningMotorID = 3
    backRightDrivingMotorReversed = True
    backRightTurningMotorReversed = False
    backRightAbsoluteEncoderId = 15 #Back Right
    backRightAbsoluteEncoderOffset = 2*math.pi/6 + math.pi/4 + math.pi #math.pi/2 - 0.379
    backRightAbsoluteEncoderReversed = True
    
    backLeftDrivingMotorID = 7
    backLeftTurningMotorID = 5
    backLeftDrivingMotorReversed = True #False
    backLeftTurningMotorReversed = False
    backLeftAbsoluteEncoderId = 13 # Back Left
    backLeftAbsoluteEncoderOffset = 0.4543435 + math.pi/2 + math.pi
    backLeftAbsoluteEncoderReversed = True

    
    climberSpeed = 0.75
    algaeIntakeSpeed = .5
    algaeLiftSpeed = .5
    coralLiftSpeed = .5
    coralDepositSpeed = .5

    coralL1Time = 3 # Time in seconds to reach L1 when the coral lift is running
    coralL2Time = 4 #! Test these
    coralL3Time = 5 #change value for desired time 


    #what is the fastest speed laterally our robot can go
    kphysicalMaxSpeedMetersPerSecond = 1.165 * 5 #! Find through test, current is test based on 1/2 speeds
    #or maybethrough the max rpm of motors 
    #what is the fastest speed  rotationally our robot can go
    kPhysicalMaxAngularSpeedRadiansPerSecond  = kphysicalMaxSpeedMetersPerSecond * 2 / kWheelDiameterMeters #! Find through Current UNKNOWN
    
    kTeleopDriveMaxAccelerationMetersPerSecSquared = 3 #* Found through .5 test, should be the same no matter the power
    kTeleopDriveMaxAngularAccelerationRadiansPerSecSquared = kTeleopDriveMaxAccelerationMetersPerSecSquared * 2 / kWheelDiameterMeters

    #what is the max speed we allow teleop driver to move laterally
    kTeleopDriveMaxSpeedMetersPerSecond = kphysicalMaxSpeedMetersPerSecond *1 # the /2 is the restriction we want to put on speed
    kTeleopDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * (3/4)

    # ~ Intake Constants (minicim)
    coralIntakeMotor1ID = 49 #! change to actual values
    coralIntakeMotor2ID = 50
    algaeIntakeMotor1ID = 51 #! change to actual values (all of these 49-55)
    algaeIntakeMotor2ID = 52
    liftMotorID = 53
    climbingMotor1ID = 54
    climbingMotor2ID = 55


    # # ~ Midstage Constants (minicim)
    # midstageMotor1ID = 59

    # # ~ Outtake Constants (NEO)
    # shooterMotor1ID = 55 
    # shooterMotor2ID = 56
    # rotatingMotorID = 63 #! not used delete!!
    # rotatingMotorRevPerArmDegree = 1 #! must be found once shooter is made
    # kPShooterAngle = .1

    # shooterMaxLimitSwitchID = 5
    # shooterMinLimitSwitchID = 6
    
    # subwooferDistance = 0.92 + 0.29 #subwoofer distance + pivot-bumper distance
    # #height of the speaker - height of the outtake
    # speakerHeight = 2.1082 - 0.3 #! the 2.1082 might be 1.98 also. there are 2 values (look at speaker video)
    #                              #! to get the most accurate result, we need the 
    #                              #! length of shooter * sin(launch angle). this is
    #                              #! a bit difficult to get bc we don't know the launch angle.
    # #radius of the fly wheels 6ft 6in. ~198 cm
    # flyWheelRadius = 0.051 

    # flyWheelPower1 = -1 #from 0-1. like what you'd do for a motor
    # flyWheelPower2 = -0.75
    # gravityConstant = 9.8
    # #time it takes for one full rotation of the fly wheels
    # period = 60/(5676 * flyWheelPower1)
    
    # ringInitialVelocity = (2 * math.pi * flyWheelRadius) / period #in m/s

    # backupShooterAngle = math.atan(speakerHeight / subwooferDistance)

    # speakerXPosition = 8.3 #length in meters
    # speakerYPosition = 1.45 #length in meters
    # # ~ Climber Constants (NEO)
    # climbingMotorID1 = 57 #Gotta change these
    # climbingMotorID2 = 57
    #climbingMotorRightID = 58
    #climbingMotorLeftLimitSwitchID = 
    # climbingMotorRightLimitSwitchID =    

    coralMotorSpeed = .5
    coralMotorLiftSpeed = .5
    #names are in relation to back of robot similar to the swerve modules

# ~ Swerve Constants

#todo Change above
class ModuleConstants:
    kWheelDiameterMeters = UtilCommands.inchesToMeters(4) #^ wheele listed as "Wheel, Billet, 4"OD x 1.5"W (MK4/4i)"" I think that's what the 4 OD means

    kDriveMotorGearRatio = 1/6.75 # ask design
    kTurningMotorGearRatio = 1/13.3714 #*found through wcp site https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options#:~:text=35%3A1%20or-,13.3714,-%3A1

    kDrivingEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    KDrivingEncoderRPM2MeterPerSec = kDrivingEncoderRot2Meter / 60 #* Initially in RPM, converted to Mps

    kWheelDistancePerRadian = kDrivingEncoderRot2Meter / (2 * math.pi)

    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60 

    kPTurning = .2 #? turning PID controller per wheel

# ~ Auto Constants

class AutoConstants:
        kMaxSpeedMetersPerSecond = RobotConstants.kphysicalMaxSpeedMetersPerSecond / 4
        kMaxAccelerationMetersPerSecondSquared = 3
        kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4

        kPXController = 1.0
        kPYController = 1.0

        kMaxAngularSpeedRadiansPerSecond = RobotConstants.kphysicalMaxSpeedMetersPerSecond / 10

        kPThetaController = 1.0
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared,
        )

        #^Added this today (1/11/2024)
        
        # pathFollowerConfig = PPHolonomicPathFollowerConfig( 
        #         PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
        #         PIDConstants(0.0, 0.0, 0.0),#PIDConstants(.9, 0.0, 0.07), # Rotation PID constants , # PIDConstants(9.0, 0.0, 0.5),
        #         kMaxSpeedMetersPerSecond, # Max module speed, in m/s
        #         math.sqrt(RobotConstants.kTrackWidth**2 + RobotConstants.kWheelBase**2), # Drive base radius in meters. Distance from robot center to furthest module.
        #         ReplanningConfig() # Default path replanning config. See the API for the options here
        #     )

        constraints = PathConstraints(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared,
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared
        )

# class sim:
#     kSimTargetName = "SimTarget"
#     # kSimDefaultTargetLocation = Pose2d(
#     #     kFieldLength / 2, kFieldWidth / 2, 180 * kRadiansPerDegree
#     # )
#     """[meters, meters, radians]"""

#     kSimDefaultRobotLocation = Pose2d(FildConstants.kFieldLength / 2, FildConstants.kFieldWidth / 2, 0)
#     # kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in
#     # kSimBallName = "SimBall"
#     # kSimDefaultBallLocation = Pose2d(kFieldLength / 4, kFieldWidth / 2, 0)

#     """meters"""

#     kSimRobotPoseArrayKey = "SimRobotPoseArray"
#     kSimTargetPoseArrayKey = "SimTargetPoseArray"
#     kSimBallPoseArrayKey = "SimBallPoseArray"
#     kSimTargetHeightKey = "SimTargetHeight"
#     kSimTargetTrackingModuleName = "sim_target_tracker"
#     kSimTargetUpperHubRadius = 2

#     kSimFrontLeftDriveMotorPort = 0
#     kSimFrontLeftSteerMotorPort = 1
#     kSimFrontRightDriveMotorPort = 2
#     kSimFrontRightSteerMotorPort = 3
#     kSimBackLeftDriveMotorPort = 4
#     kSimBackLeftSteerMotorPort = 5
#     kSimBackRightDriveMotorPort = 6
#     kSimBackRightSteerMotorPort = 7


#     kSimFrontLeftDriveEncoderPorts = (16, 1)
#     kSimFrontLeftSteerEncoderPorts = (2, 3)
#     kSimFrontRightDriveEncoderPorts = (4, 5)
#     kSimFrontRightSteerEncoderPorts = (6, 7)
#     kSimBackLeftDriveEncoderPorts = (8, 9)
#     kSimBackLeftSteerEncoderPorts = (10, 11)
#     kSimBackRightDriveEncoderPorts = (12, 13)
#     kSimBackRightSteerEncoderPorts = (14, 15)
