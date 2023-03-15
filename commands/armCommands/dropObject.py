from subsystems.armsubsystem import ArmSubsystem
import commands2


class dropObject(commands2.CommandBase):
    def __init__(self, angle, arm: ArmSubsystem) -> None:
        super().__init__()
        self.angle = angle
        self.arm= arm

    def initialize(self):
        pass

    def execute(self) -> None:
        speed = 0.3
        angle = 90 #This is a random guess fix later
        self.arm.setGrabbingArmAngle(self.angle,speed)

    def end(self, interrupted: bool) -> None:
        self.arm.setGrabbingArmSpeedWithAuto(0)

    def isFinished(self) -> bool:
        return (self.angle - self.arm.tolerance <= self.arm.grabbingArmEncoderDegrees or self.angle + self.arm.tolerance >= self.arm.grabbingArmEncoderDegrees)