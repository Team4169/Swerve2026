from subsystems.climbingsubsystem import ClimbingSubsystem
import commands2


class MoveLiftArmPastLocation(commands2.CommandBase):
    def __init__(self, tickLocation: int, above: bool, power: float, climb: ClimbingSubsystem) -> None:
        super().__init__()
        self.tickLocation = tickLocation
        self.above = above
        self.power = power
        self.climb = climb

    def initialize(self):
        pass

    def execute(self) -> None:
        self.climb.setLiftArm(self.power)

    def end(self, interrupted: bool) -> None:
        self.climb.setLiftArm(0)

    def isFinished(self) -> bool:
        if self.climb.getLiftArmLimitSwitchPressed():
            return True
        if self.above:
            if self.climb.getLiftArmEncoderDistance() > self.tickLocation:
                return True
        else:
            if self.climb.getLiftArmEncoderDistance() < self.tickLocation:
                return True
        return False
