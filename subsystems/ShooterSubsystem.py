# import commands2
# import wpilib
# import wpilib.drive
# import constants
# from constants import RobotConstants
# import ntcore
# import rev
# import math
# from wpimath.controller import PIDController
# from wpimath.geometry import Rotation2d


# class ShooterSubsystem(commands2.SubsystemBase):
#     def __init__(self) -> None:
#         super().__init__()
#         # commands2.SubsystemBase.__init__(self)
#         # ~ smartdashboard
#         self.sd = wpilib.SmartDashboard        
        
#         #* shooting motors
#         self.shooterMotor1 = rev.CANSparkMax(RobotConstants.shooterMotor1ID, rev.CANSparkLowLevel.MotorType.kBrushless)
#         self.shooterMotor2 = rev.CANSparkMax(RobotConstants.shooterMotor2ID, rev.CANSparkLowLevel.MotorType.kBrushless)

#         self.rotatingMotor = rev.CANSparkMax(RobotConstants.rotatingMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)

#         #* encoders
#         self.rotatingMotorEncoder = self.rotatingMotor.getEncoder()

#         self.rotatingMotorDegrees = float(self.rotatingMotorEncoder.getPosition() / RobotConstants.rotatingMotorRevPerArmDegree)

#         #* shooterAngle PID controller
#         self.shooterAnglePIDController = PIDController(RobotConstants.kPShooterAngle, 0, 0)

#         #* limit Switches
#         # self.shooterMaxLimitSwitch = wpilib.DigitalInput(RobotConstants.shooterMaxLimitSwitchID)
#         # self.shooterMinLimitSwitch = wpilib.DigitalInput(RobotConstants.shooterMinLimitSwitchID)

#         self.network_tables = ntcore.NetworkTableInstance.getDefault()
#         self.camera_tables = self.network_tables.getTable("SmartDashboard")

#     def getShooterAngle(self): 
#         self.jetson1rotation = self.camera_tables.getEntry("r1").getValue()
#         self.jetson2rotation = self.camera_tables.getEntry("r2").getValue()
#         self.jetson1X = self.camera_tables.getEntry("x1").getValue()
#         self.jetson2X = self.camera_tables.getEntry("x2").getValue()
#         self.jetson1Y = self.camera_tables.getEntry("y1").getValue()
#         self.jetson2Y = self.camera_tables.getEntry("y2").getValue()
#         self.jetson1weight = self.camera_tables.getEntry("w1").getValue()
#         self.jetson2weight = self.camera_tables.getEntry("w2").getValue()
        
#         self.rotationAve = math.atan2((math.sin(self.jetson1rotation) * self.jetson1weight + math.sin(self.jetson2rotation) * self.jetson2weight) / (self.jetson1weight + self.jetson2weight),
#                    (math.cos(self.jetson1rotation) * self.jetson1weight + math.cos(self.jetson2rotation) * self.jetson2weight) / (self.jetson1weight + self.jetson2weight)) % (2 * math.pi)
#         self.xAve = (self.jetson1X * self.jetson1weight + self.jetson2X * self.jetson2weight) / (self.jetson1weight +self.jetson2weight)
#         self.yAve = (self.jetson1Y * self.jetson1weight + self.jetson2Y * self.jetson2weight) / (self.jetson1weight +self.jetson2weight)
        
#         self.xDistance = self.RobotConstants.speakerToCenterOFField - self.xAve
#         self.yDistance = self.RobotConstants.heightoFField - self.yAve
#         self.distanceToShooter = math.sqrt(self.xDistance**2 + self.yDistance**2)
        
#         #returns theta value for our shooter given distance from speaker
#         velocity = RobotConstants.ringInitialVelocity
#         height = RobotConstants.speakerHeight
#         gravity = RobotConstants.gravityConstant
#         if self.distanceToShooter == 0:
#             return float(0)
#             #return Rotation2d(0)
#         theta = math.atan((velocity**2) - math.sqrt((velocity**4) - gravity*((gravity*(self.distanceToShooter**2)) + (2*height*(velocity**2))))/(gravity*self.distanceToShooter))
#         self.sd.putNumber('Theta', theta)
#         return float(theta) 
#         #return Rotation2d(theta)

#     def setShooterAngle(self, theta) -> None:
#         self.rotatingMotorDegrees = float(self.rotatingMotorEncoder.getPosition() / RobotConstants.rotatingMotorRevPerArmDegree)
#         self.rotatingMotor.set(self.shooterAnglePIDController.calculate(self.rotatingMotorDegrees, theta))
        
        
#     def runShooter(self):
#         self.shooterMotor1.set(RobotConstants.flyWheelPower1)
#         self.shooterMotor2.set(RobotConstants.flyWheelPower2)

#     def stopShooter(self):
#         self.shooterMotor1.set(0)
#         self.shooterMotor2.set(0)
        
#     def stopRotating(self):
#         self.rotatingMotor.set(0)
    
#     def getForwardLimitSwitch(self):
#         self.shooterMaxLimitSwitch.get()

#     def getReverseLimitSwitch(self):
#         self.shooterMinLimitSwitch.get()

#     def rotateManually(self, speed):
#         self.rotatingMotor.set(speed)
    