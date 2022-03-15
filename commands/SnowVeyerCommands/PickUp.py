from subsystems.snowveyorsubsystem import SnowveyorSubsystem
import commands2
import wpilib
from networktables import NetworkTables

class pickUp(commands2.CommandBase):
    def __init__(self, duration: float, speed:float, snowveyor: SnowveyorSubsystem) -> None:
        super().__init__()
        self.sd = NetworkTables.getTable("SmartDashboard")
        self.snowveyor = snowveyor
        self.speed = speed
        self.duration = duration
        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.snowveyor.tankDrive(self.speed, 0)

    def end(self, interrupted: bool) -> None:
        self.snowveyor.tankDrive(0, 0)

    def isFinished(self) -> bool:
        self.sd.putValue("timer for intake", self.timer.get())
        self.sd.putValue("duration for intake", self.timer.get())
        return self.timer.get() > self.duration