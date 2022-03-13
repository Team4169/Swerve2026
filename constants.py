# Motors
leftTalon = 1
leftVictor = 2
rightTalon = 7
rightVictor = 6

# Encoders
kLeftEncoderPorts = (0, 1)
kRightEncoderPorts = (2, 3)
kLeftEncoderReversed = False
kRightEncoderReversed = True

tpf = -924


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
liftArmDownLimitSwitch = 1
liftArmDownLimitSwitchPressedValue = False
rotateArmBackLimitSwitch = 2
rotateArmBackLimitSwitchPressedValue = False
rotateArmRobotLimitSwitch = 3
rotateArmRobotLimitSwitchPressedValue = False


deadzone = 0.1