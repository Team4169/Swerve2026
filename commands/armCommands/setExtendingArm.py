from subsystems.armsubsystem import ArmSubsystem
import commands2



class setExtendingArm(commands2.CommandBase):
    def __init__(self, distance, arm: ArmSubsystem) -> None:
        super().__init__()
        self.distance = distance
        self.arm= arm

    def initialize(self):
        pass

    def execute(self) -> None:
        percent = (20/self.distance) * 100
        speed = 0.3 #expirement with this
        self.arm.setExtendingArmPercentWithAuto(percent,speed)

    def end(self, interrupted: bool) -> None:
        self.arm.setGrabbingArmSpeedWithAuto(0)

    def isFinished(self) -> bool:
        return (percent + self.arm.tolerance >= self.arm.extendingArmEncoderPercent)