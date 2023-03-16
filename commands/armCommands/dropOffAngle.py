from subsystems.armsubsystem import ArmSubsystem
import commands2
from .setRotatingArm import setRotatingArm

class dropOffAngle(commands2.CommandBase):
    def __init__(self, distance, height, arm: ArmSubsystem) -> None:
        super().__init__()
        self.distance = distance
        self.height = height
        self.arm= arm

    def initialize(self):
        pass

    def execute(self) -> None:
        speed = 0.3
        angle = self.arm.dropOffAngleAuto(self.distance,self.height)
        self.arm.setRotatingArmAngle(angle,speed)

    def end(self, interrupted: bool) -> None:
        self.arm.setRotatingArmSpeed(0)

    def isFinished(self) -> bool:
        return (self.angle - self.arm.tolerance <= self.arm.rotatingArmEncoderDegrees or self.angle + self.arm.tolerance >= self.arm.rotatingArmEncoderDegrees)