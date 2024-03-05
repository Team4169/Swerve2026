import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math
import phoenix5

class IntakeSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard

        #* Intake motors
        self.intakeMotor1 = phoenix5.WPI_TalonSRX(RobotConstants.intakeMotor1ID)
        self.intakeMotor2 = phoenix5.WPI_TalonSRX(RobotConstants.intakeMotor2ID)

    def runIntake(self, speed: float):
        self.intakeMotor1.set(speed)
        self.intakeMotor2.set(speed)
    
    def stopIntake(self):
        self.intakeMotor1.set(0)
        self.intakeMotor2.set(0)