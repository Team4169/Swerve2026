import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d


class ShooterSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard        
        
        #* shooting motors
        self.shooterMotor1 = rev.CANSparkMax(RobotConstants.shooterMotor1ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterMotor2 = rev.CANSparkMax(RobotConstants.shooterMotor2ID, rev.CANSparkLowLevel.MotorType.kBrushless)

        self.rotatingMotor = rev.CANSparkMax(RobotConstants.rotatingMotor1ID, rev.CANSparkLowLevel.MotorType.kBrushless)

        #* encoders
        # self.shooterMotorEncoder = self.shooterMotor.getEncoder()
        self.rotatingMotorEncoder = self.rotatingMotor.getEncoder()

        self.rotatingMotorDegrees = self.rotatingMotorEncoder.getPosition() / RobotConstants.rotatingMotorRevPerArmDegree

        #* shooterAngle PID controller
        self.shooterAnglePIDController = PIDController(RobotConstants.kPShooterAngle, 0, 0)

        #* limit Switches
        #self.rotatingMotorEncoder = rotatingMotorEncoder
        #!this is code the luc wrote, but could very much be wrong
        self.shooterMaxLimitSwitch = wpilib.DigitalInput(RobotConstants.shooterMaxLimitSwitchID)
        self.shooterMinLimitSwitch = wpilib.DigitalInput(RobotConstants.shooterMinLimitSwitchID)

        #! this is the code that was written before, which luc commented out when he wrote the code above
        # self.shooterMaxLimitSwitch = self.rotatingMotorEncoder.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # self.shooterMinLimitSwitch = self.rotatingMotorEncoder.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)

    def getShooterAngle(self, shooter_distance) -> Rotation2d(): 
        #returns theta value for our shooter given distance from speaker
        velocity = RobotConstants.ringInitialVelocity
        height = RobotConstants.speakerHeight
        gravity = RobotConstants.gravityConstant
        if shooter_distance == 0:
            return Rotation2d(0)
        theta = math.atan((velocity**2) - math.sqrt((velocity**4) - gravity*((gravity*(shooter_distance**2)) + (2*height*(velocity**2))))/(gravity*shooter_distance))
        self.sd.putNumber('Theta', theta)
        return Rotation2d(theta)    

    def setShooterAngle(self, theta) -> None:
        self.rotatingMotor.set(self.shooterAnglePIDController.calculate(self.rotatingMotorDegrees, theta))

    def runShooter(self):
        self.shooterMotor1.set(RobotConstants.flyWheelPower)
        self.shooterMotor2.set(RobotConstants.flyWheelPower)

    def stopShooter(self):
        self.shooterMotor1.set(0)
        self.shooterMotor2.set(0)
        

    def stopRotating(self):
        self.rotatingMotor.set(0)
    
    def getForwardLimitSwitch(self):
        self.shooterMaxLimitSwitch.get()

    def getReverseLimitSwitch(self):
        self.shooterMinLimitSwitch.get()

    def rotateManually(self, speed):
        self.rotatingMotor.set(speed)
    
    