import commands2
import math

from subsystems.drivesubsystem import DriveSubsystem


class MoveCommandSpeed(commands2.CommandBase):
    def __init__(self, distance: float, heading: float, drive: DriveSubsystem) -> None:
        super().__init__()
        # Feature to add - difference tolerance per command instance. Currently uses the default from DriveSubsystem
        # Feature to add - different max speed for each command. Currently uses method of DriveSubsystem.
        self.drive = drive
        self.targetTicks = distance * self.drive.tpf
        self.heading = heading
        # print("distance goal", distance)
        # print("turn goal", heading)
        self.goal_threshold_ticks = 100 # I believe 50 ticks per second, confirm.
        # self.addRequirements(drive)

    def initialize(self) -> None:
        self.drive.resetEncoders()
        # This increases everytime the robot remains in the target
        self.in_threshold = 0
        # self.drive.driveController.setSetpoint(self.distance)
        # self.drive.turnController.setSetpoint(self.heading)

    def execute(self) -> None:
        self.currDistance = self.drive.leftTalon.getSelectedSensorPosition()
        self.distanceToTarget = self.targetTicks - self.currDistance
        self.distanceToTargetFeet = self.distanceToTarget / self.drive.tpf
        k = 15
        sign = abs(self.distanceToTarget)/self.distanceToTarget
        self.speed = self.drive.maxDriveSpeed # 0.5 * self.drive.maxDriveSpeed * (math.tanh((5 * self.distanceToTargetFeet - sign * k)/5) + sign * 1)
        # print('we done furreal')
        # self.drive.sd.putValue("Gyro Yaw", self.drive.gyro.getYaw())
        # self.drive.sd.putValue("distance goal new", self.distance)
        # self.drive.sd.putValue("turn goal", self.heading)
        # self.drive.sd.putValue("average ticks", self.drive.getAverageEncoderTicks())
        self.drive.driveMecanum(self.speed,0,0)

    def end(self, interrupted: bool) -> None:
        self.drive.driveMecanum(0, 0, 0)

    def isFinished(self) -> bool:
        # print("stopped move with speed")
        return (abs(self.targetDistance - self.drive.leftTalon.getSelectedSensorPosition()) < self.goal_threshold_ticks)
        
