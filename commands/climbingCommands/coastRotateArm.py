from subsystems.climbingsubsystem import ClimbingSubsystem
import commands2
 

class coastRotateArm(commands2.CommandBase):
    def __init__(self, isCoast, climb: ClimbingSubsystem) -> None:
        super().__init__()
        self.isCoast = isCoast
        self.climb = climb



    def initialize(self):
        pass

    def execute(self) -> None:
        self.climb.setCoast(isCoast)
        output("Break Mode:",isCoast)

    def end(self, interrupted: bool) -> None:
        self.climb.setCoast(False)
