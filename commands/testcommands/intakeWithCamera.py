import commands2
from subsystems.swervesubsystem import SwerveSubsystem
# from subsystems.midstageSubsystem import MidstageSubsystem
# from subsystems.intakeSubsystem import IntakeSubsystem

from constants import RobotConstants
from subsystems.swervemodule import SwerveModule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import time
import ntcore
class move1module(commands2.CommandBase):
    def __init__(self, swerve: SwerveSubsystem) -> None: # intake:IntakeSubsystem, midstage:MidstageSubsystem
        super().__init__()
        self.swerve = swerve
        # self.intake = intake
        # self.midstage = midstage
        self.network_tables = ntcore.NetworkTableInstance.getDefault()
        self.camera_tables = self.network_tables.getTable("SmartDashboard")

    def initialize(self):
        # self.intake.runIntake(.5)
        # self.midstage.runMidstage(.5)
        pass
    def execute(self) -> None:
        pass
    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return self.camera_tables.getEntry("inMid").getBoolean(False)
