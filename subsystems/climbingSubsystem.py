import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math

class ClimbingSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard

        #* Climbing motor
        self.ClimbingMotor1 = rev.SparkMax(RobotConstants.climbingMotor1ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.ClimbingMotor2 = rev.SparkMax(RobotConstants.climbingMotor2ID, rev.SparkLowLevel.MotorType.kBrushless)
    
    def runClimbingMotors(self, speed: float):
        self.ClimbingMotor1.set(speed)
        self.ClimbingMotor2.set(speed)

    def stopClimbingMotors(self):
        self.ClimbingMotor1.set(0)
        self.ClimbingMotor2.set(0)
