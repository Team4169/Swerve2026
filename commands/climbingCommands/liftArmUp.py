from subsystems.climbingsubsystem import ClimbingSubsystem
import commands2
import wpilib

class liftArmUp(commands2.CommandBase):
    def __init__(self, power: float, climb: ClimbingSubsystem) -> None:
        super().__init__()
        self.power = power
        self.climb = climb

    def initialize(self):
        pass

    def execute(self) -> None:
        self.climb.setLiftArm(self.power)

    def end(self):
        self.climb.setLiftArm(0)
