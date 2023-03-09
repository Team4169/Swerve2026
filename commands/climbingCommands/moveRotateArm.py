#from subsystems.climbingsubsystem import ClimbingSubsystem
import commands2


class MoveRotateArm(commands2.CommandBase):
    def __init__(self, power: float, climb: ClimbingSubsystem) -> None:
        super().__init__()
        self.power = power
        self.climb = climb

    def initialize(self):
        pass

    def execute(self) -> None:
        self.climb.setRotateArm(self.power)

    def end(self, interrupted: bool) -> None:
        self.climb.setRotateArm(0)

    def isFinished(self) -> bool:
        if self.climb.getRotateArmLimitSwitchPressed() and self.power > 0:
            return True