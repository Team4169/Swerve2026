import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math

class climbingSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard

        #* Climbing motor
        self.ClimbingMotor1 = rev.SparkMax(RobotConstants.climbingMotor1ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.ClimbingMotor2 = rev.SparkMax(RobotConstants.climbingMotor2ID, rev.SparkLowLevel.MotorType.kBrushless)
    
    def runClimbingMotor1(self, speed: float):
        self.ClimbingMotor1.set(speed)

    def stopClimbingMotor1(self):
        self.ClimbingMotor1.set(0)
    
    def runClimbingMotor2(self, speed: float):
        self.ClimbingMotor2.set(speed)

    def stopClimbingMotor2(self):
        self.ClimbingMotor2.set(0)
    