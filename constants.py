#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
#

import math
import wpilib

# Motors
leftTalon = 3
leftTalon2 = 7
rightTalon = 9
rightTalon2 = 4

# Arm motors

extendingArmID = 11 
rotatingArmID = 12 
grabbingArmID = 13

# Arm encoders
grabbingArmEncoderPort = 0
positiveTicksPerDeg = 173/360
negativeTicksPerDeg = 221500/360


#todo find out how many revolutions per arm length and replace none with that num
extendingArmRevPerArmPercent = 100 # / 100
rotatingArmRevPerArmDegree = 100 # / 360


# *Arm pickup systems
testDistance  = 21 #! this would be in place for the actual distance from the AI code
cameraDistanceFromArm = 20  #in meters (20 is inches), (39.37 is inches per meter)
piviotDistanceFromGround = 5.957 # in inches
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

# Physical parameters
kDriveTrainMotorCount = 2
kTrackWidth = 0.381 * 2
kGearingRatio = 8
kWheelRadius = 0.0508

# kEncoderResolution = -


#SnowVeyor
intake = 10
outtake = 12

# Climbing
liftArm = 5
rotateArm = 4


liftArmSlowSpeed = .1
liftArmFastSpeed = .5
liftArmCloseToBottomTicks = -100
liftArmCloseToTopTicks = -500

rotateArmSlowSpeed = .05
rotateArmFastSpeed = .1
rotateArmCloseToRobotTicks = 50
rotateArmCloseToBackTicks = 100

deadzone = .1

maxBalanceAngle = 15
balanceSensitivity = -2
maxBalanceSpeed = .3

#Bottom (leftmost) cube drop off is origin
startPos = [3.4,1.7,0]
endPos = [2.43,1.73,0.93]
balanceDistance = 0.8
cubeToConeDistance = 0.58