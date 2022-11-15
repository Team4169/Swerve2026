#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
#

import math
import wpilib

# Motors
leftTalon = 6
leftTalon2 = 10
rightTalon = 8
rightTalon2 = 2

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
kSnowveyorControllerPort = 1

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

liftArmUpLimitSwitch = 0
liftArmUpLimitSwitchPressedValue = False
liftArmDownLimitSwitch = 2
liftArmDownLimitSwitchPressedValue = False
rotateArmBackLimitSwitch = 3
rotateArmBackLimitSwitchPressedValue = False
rotateArmRobotLimitSwitch = 1
rotateArmRobotLimitSwitchPressedValue = False

liftArmSlowSpeed = .1
liftArmFastSpeed = .5
liftArmCloseToBottomTicks = -100
liftArmCloseToTopTicks = -500

rotateArmSlowSpeed = .05
rotateArmFastSpeed = .1
rotateArmCloseToRobotTicks = 50
rotateArmCloseToBackTicks = 100

deadzone = .1