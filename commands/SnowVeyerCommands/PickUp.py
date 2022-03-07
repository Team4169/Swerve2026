from subsystems.drivesubsystem import DriveSubsystem
import commands2
import wpilib
#time in seconds
class pickUp(commands2.CommandBase):    
    def __init__(self, duration: float, speed:float) -> None:
        super().__init__()
        self.speed = speed
        self.duration = duration
        self.timer = wpilib.Timer()
    
    def initialize(self):
        DriveSubsystem.snowveyor.tankDrive(self.speed, 0)
        self.timer.reset()
        self.timer.start()
    
    def execute(self) -> None:
        pass

    def end(self):
        DriveSubsystem.snowveyor.tankDrive(0, 0)
    
    def isFinished(self) -> bool:
        return self.timer.get() > self.duration