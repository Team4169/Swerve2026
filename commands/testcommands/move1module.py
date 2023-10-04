import commands2
from subsystems.swervesubsystem import SwerveSubsystem

class move1module(commands2.CommandBase):
    def __init__(self, distance, height, swerve: SwerveSubsystem) -> None:
        super().__init__()
        self.swerve = swerve

    def initialize(self):
        pass

    def execute(self) -> None:
       pass 

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return 