import commands2

from subsystems.drivesubsystem import DriveSubsystem


class moveTillGyro(commands2.CommandBase):
    def __init__(self, drive: DriveSubsystem, arm) -> None:
        super().__init__()
        self.drive = drive
        self.arm = arm
    def end(self, interrupted: bool) -> None:
        self.drive.driveMecanum(0,0,0)
    
    def execute(self) -> None:
        if self.drive.gyro.getPitch() > -3 and self.drive.gyro.getPitch() < 2:
            self.drive.driveMecanum(.6, 0, 0)
        elif self.drive.gyro.getpitch() < -7:
            self.drive.driveMecanum(.3, 0,  0)

            


    def isFinished(self) -> bool:
        # self.arm.shouldMove = True
        return (self.drive.gyro.getPitch() < -11) 