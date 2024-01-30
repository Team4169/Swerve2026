import commands2
from subsystems.swervesubsystem import SwerveSubsystem
from constants import RobotConstants
from subsystems.swervemodule import swervemodule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from constants import *
import time
from wpimath.trajectory import TrajectoryConfig, Trajectory, TrajectoryUtil, TrajectoryGenerator, TrajectoryParameterizer

from wpimath.controller import ProfiledPIDController, PIDController, ProfiledPIDControllerRadians

from commands2 import SwerveControllerCommand

class octagon(commands2.SequentialCommandGroup):
    def __init__(self, swerve: SwerveSubsystem) -> None:
        super().__init__()
        self.swerve = swerve

        # # 1. Create Trajectory settings
        self.trajectoryConfig = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        self.trajectoryConfig.setKinematics(RobotConstants.kDriveKinematics)

        # 2. Generate Trajectory
         #make the robot go in a octagon
        self.octagonTrajectory = TrajectoryGenerator.generateTrajectory(
            # ? initial location and rotation
            Pose2d(0, 0, Rotation2d(0)),
            [
                # ? points we want to hit
                Translation2d(1, 0),
                Translation2d(2, 1),
                Translation2d(2, 2),
                Translation2d(1, 3),
                Translation2d(0, 3),
                Translation2d(-1, 2),
                Translation2d(-1, 1),
                
            ],
            # ? final location and rotation
            Pose2d(0, 0, Rotation2d(180)),
            self.trajectoryConfig
        )
                # 3. Create PIdControllers to correct and track trajectory
        self.xController = PIDController(AutoConstants.kPXController, 0, 0)
        self.yController = PIDController(AutoConstants.kPYController, 0, 0)
        self.thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints)
        self.thetaController.enableContinuousInput(-math.pi, math.pi)

        # 4. Construct command to follow trajectory
        self.swerveControllerCommand = SwerveControllerCommand(
           self.octagonTrajectory,
           self.swerve.getPose,
           RobotConstants.kDriveKinematics,
           RobotConstants.HolonomicPathFollowerConfig,
           self.swerve.setModuleStates,
           [self.swerve]
        )
        # 5. Add some init and wrap up, and return command 
        self.octagon = commands2.SequentialCommandGroup(
            commands2.InstantCommand(lambda:self.swerve.resetOdometry(self.trajectory.initialPose())),
            self.swerveControllerCommand,
            commands2.InstantCommand(lambda:self.swerve.stopModules())
        )
    def getCommand(self):
        return self.octagon