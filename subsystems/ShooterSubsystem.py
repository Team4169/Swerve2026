import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math
from wpimath.controller import PIDController

class ShooterSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard        
        
        #* shooting motors
        self.shooterMotor1 = rev.CANSparkMax(RobotConstants.shooterMotor1ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rotatingMotor = rev.CANSparkMax(RobotConstants.rotatingMotor1ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        #* encoders
        self.shooterMotor1Encoder = self.shooterMotor1.getEncoder()
        self.rotatingMotor1Encoder = self.rotatingMotor1.getEncoder()

        self.rotatingMotorDegrees = self.rotatingMotor1Encoder.getPosition() / RobotConstants.rotatingMotorRevPerArmDegree

        #* shooterAngle PID controller
        self.shooterAnglePIDController = PIDController(RobotConstants.kPShooterAngle, 0, 0)

        #* limit Switches
        self.shooterMaxLimitSwitch = self.rotatingMotor1Encoder.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.shooterMaxLimitSwitch = self.rotatingMotor1Encoder.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)

    def getShooterAngle(self, shooter_distance) -> tuple(float, float): 
        #returns theta value for our shooter given distance from speaker
        #! this needs to be checked! I have extreemely low faith in this working
        theta = -math.asin((7 * (7 * RobotConstants.ringInitialVelocity - math.sqrt((40 * RobotConstants.speakerHeight + 49) * RobotConstants.ringInitialVelocity**2))) / (20 * RobotConstants.ringInitialVelocity**2)) #+2*math.pi
        return theta

    def setShooterAngle(self, theta) -> None:
        self.rotatingMotor.set(self.shooterAnglePIDController.calculate(self.rotatingMotorDegrees, theta))


    def runShooter(self):
        self.shooterMotor1.set(0.8)

    def stopShooter(self):
        self.shooterMotor1.set(0)

    def stopRotating(self):
        self.rotatingMotor.set(0)
    

