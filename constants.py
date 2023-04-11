#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
#

import math
import wpiutil
import UtilCommands
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry._geometry import Translation2d
# Motors
leftTalon = 3
leftTalon2 = 7
rightTalon = 9
rightTalon2 = 4

moveRestriction = .75
# Arm motors

extendingArmID = 11 
rotatingArmID = 12 
grabbingArmID = 13

# Arm encoders
grabbingArmEncoderPort = 0
positiveTicksPerDeg = 173/360
negativeTicksPerDeg = 221500/360

#todo: find the degrees that this should be 
startingRotatingDegrees = 50

#! extending arm give negative power as "foreward"
extendingArmRevPerArmPercent = -103.47 / 100
rotatingArmRevPerArmDegree = - 59 / 68 # / 360


# *Arm pickup systems
testDistance  = 21 #! this would be in place for the actual distance from the AI code
cameraDistanceFromArm = 20  #in meters (20 is inches), (39.37 is inches per meter)
pivotDistanceFromGround = 5.957 # in inches
armPickupHeight = 3.427 # in inches
maxArmLength = 68 # in inches
minArmLength = 35 # in inches
lowerArmAngleLimit = -7



# Autonomous
kAutoDriveDistanceInches = 60
kAutoBackupDistanceInches = 20
kAutoDriveSpeed = 0.2

# Operator Interface
kDriverControllerPort = 0
kArmControllerPort = 1

# kEncoderResolution = -

deadzone = .1

maxBalanceAngle = 15
balanceSensitivity = -2.2
maxBalanceSpeed = .3

#Bottom (leftmost) cube drop off is origin
startPos = [0,67,133] # 0 67 133
endPos = [37,68,110] # 37 68 96
balanceDistance = 31
cubeToConeDistance = 22.8
dropOffDistance = 58 #Test This Distance
cubeTargetHeights = [2,23,35]
coneTargetHeights = [2,37,50]


#robot specifications
kTrackWidth = UtilCommands.inchesToMeters(30) #todo: find the actual track width 
    # Distance between the right and left wheels
kWheelBase = UtilCommands.inchesToMeters(30) #todo: find the actual wheel base
    # Distance between the front and back wheels
kDriveKinematics = SwerveDrive4Kinematics(
    Translation2d(kTrackWidth / 2, kWheelBase / 2),
    Translation2d(kTrackWidth / 2, -kWheelBase / 2),
    Translation2d(-kTrackWidth / 2, kWheelBase / 2),
    Translation2d(-kTrackWidth / 2, -kWheelBase / 2),
    #location of each swerve module relative to the center of the robot
)

# Swerve Constants 


kTeleopDriveMaxAccelerationUnitsPerSec = 1 # 0 to 1, represents move restriciton
KTeleopDriveMaxAngularAccelerationUnitsPerSec = 1

kTeleopDriveMaxSpeedMetersPerSecond = 12 
kTeleopDriveMaxAngularSpeedRadiansPerSecond = 12

kphysicalMaxSpeedMetersPerSecond = 12 #? not sure how to get this, maybe look online and multiply by gear ratio?
kWheeleDiameterMeters = UtilCommands.inchesToMeters(4) #? wheele listed as "Wheel, Billet, 4"OD x 1.5"W (MK4/4i)"" I think that's what the 4 OD means

kdriveMotorGearRatio = 1/5.43 #^placeholder
kTurningMotorGearRatio = 1/5.43 #^placeholder
#todo Change above

kDrivingEncoderRot2Meter = kdriveMotorGearRatio * math.pi * kWheeleDiameterMeters
KDrivingEncoderRPM2MeterPerSec = kDrivingEncoderRot2Meter / 60

kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60 

KPturning = .5 #? turning PID controller per wheele

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