from subsystems.drivesubsystem import DriveSubsystem
import commands2
import wpilib
#time in seconds
class pickUp(commands2.CommandBase):
    def __init__(self, duration: float, speed:float, drive: DriveSubsystem) -> None:
        super().__init__()
        self.drive = drive
        self.speed = speed
        self.duration = duration
        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.drive.snowveyor.tankDrive(self.speed, 0)

    def end(self, interrupted: bool) -> None:
        self.drive.snowveyor.tankDrive(0, 0)

    def isFinished(self) -> bool:
        return self.timer.get() > self.duration