from subsystems.climbingsubsystem import ClimbingSubsystem
import commands2


class MoveLiftArmToLimitSwitch(commands2.CommandBase):
    def __init__(self, power: float, climb: ClimbingSubsystem) -> None:
        super().__init__()
        self.power = power
        self.climb = climb

    def initialize(self):
        pass

    def execute(self) -> None:
        print("excuting MoveLiftArmToLimitSwitch")
        self.climb.setLiftArm(self.power)

    def end(self, interrupted: bool) -> None:
        self.climb.setLiftArm(0)

    def isFinished(self) -> bool:
        return self.climb.getLiftArmLimitSwitchPressed()
