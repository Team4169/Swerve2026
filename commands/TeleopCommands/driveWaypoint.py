from commands2 import CommandBase
from pathplannerlib.auto import AutoBuilder
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from subsystems.swervesubsystem import SwerveSubsystem
import math
from constants import AutoConstants

class DriveWaypoint(CommandBase):
    def __init__(self, drive: SwerveSubsystem) -> None:
        super().__init__()

        self.swerve = drive

        self.command = CommandBase()

        self.running = False
        self.addRequirements(self.swerve)

    def initialize(self) -> None:
        print("initialize...initialize...initialize...initialize...initialize...initialize...")
        self.running = True
        
        changeHorizontal = 10
        changeDistance = 10

        changeRot = math.atan2(changeHorizontal, changeDistance)
        targetPose = Pose2d(changeDistance, changeHorizontal, Rotation2d.fromRotations(changeRot / (math.pi * 2)))

        self.command = AutoBuilder.pathfindToPose(
            targetPose,
            AutoConstants.constraints
        )

        self.command.initialize()

    def execute(self) -> None:
        print("execute...execute...execute...execute...execute...execute...execute...")
        self.command.execute()
        if self.command.isFinished():
            self.command.end(False)
            self.running = False

    def isFinished(self) -> bool:
        return False