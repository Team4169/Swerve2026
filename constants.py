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
grabbingArmID = 2
extendingArmID = 100 # todo find out what these are
rotatingArmID = 101  # todo

# Arm encoders
grabbingArmEncoderPort = 0
positiveTicksPerDeg = 173/360
negativeTicksPerDeg = 221500/360

extendingArmEncoderID = None
#todo find out how many revolutions per arm length and replace none with that num
extendingArmRevPerArmPercent = 100 # / 100

rotatingArmEncoderID = None




# Encoders
kLeftEncoderPorts = (0, 1)
kRightEncoderPorts = (2, 3)
kLeftEncoderReversed = False;
kRightEncoderReversed = True

kEncoderCPR = 1024
kWheelDiameterInches = 6
# Assumes the encoders are directly mounted on the wheel shafts
# kEncoderDistancePerPulse = (kWheelDiameterInches * math.pi) / kEncoderCPR
kEncoderDistancePerPulse = 1 / 924 * 12 #in inches

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

grabbingArmOpenLimitSwitch = 1
grabbingArmOpenLimitSwitchPressedValue = False

# liftArmDownLimitSwitch = 2
# liftArmDownLimitSwitchPressedValue = False
# rotateArmBackLimitSwitch = 3
# rotateArmBackLimitSwitchPressedValue = False
# rotateArmRobotLimitSwitch = 1
# rotateArmRobotLimitSwitchPressedValue = False

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