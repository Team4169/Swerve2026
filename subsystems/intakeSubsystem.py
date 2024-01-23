import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math

class IntakeSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard
        
        #* Arm motors
        self.intakeMotor1 = rev.CANSparkMax(RobotConstants.intakeMotor1ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)


        #* encoders
        self.intakeMotor1Encoder = self.intakeMotor1.getEncoder()