import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math
import phoenix5

class MidstageSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard

        #* Intake motors
        self.midstageMotor1 = rev.CANSparkMax(RobotConstants.midstageMotor1ID, rev.CANSparkLowLevel.MotorType.kBrushless)
        
        
    def runMidstage(self, speed: float):
        self.midstageMotor1.set(speed)

    def stopMidstage(self):
        self.midstageMotor1.set(0)
        
