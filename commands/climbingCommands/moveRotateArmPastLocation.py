from subsystems.climbingsubsystem import ClimbingSubsystem
import commands2


class MoveRotateArmPastLocation(commands2.CommandBase):
    def __init__(self, tickLocation: int, above: bool, power: float, climb: ClimbingSubsystem) -> None:
        super().__init__()
        self.tickLocation = tickLocation
        self.above = above
        self.power = power
        self.climb = climb

    def initialize(self):
        pass

    def execute(self) -> None:
        self.climb.setRotateArm(self.power)

    def end(self, interrupted: bool) -> None:
        self.climb.setRotateArm(0)

    def isFinished(self) -> bool:
        if self.above:
            if self.climb.getRotateArmEncoderDistance() > self.tickLocation:
                return True
        else:
            if self.climb.getRotateArmEncoderDistance() < self.tickLocation:
                return True
        return False
