#from subsystems.climbingsubsystem import ClimbingSubsystem
import commands2


class dropOffExtend(commands2.CommandBase):
    def __init__(self, distance, height, arm: ArmSubsystem) -> None:
        super().__init__()
        self.distance = distance
        self.height = height
        self.arm= arm

    def initialize(self):
        pass

    def execute(self) -> None:
        d = self.arm.dropOffExtentionAuto(self.distance,self.height)
        percent = (20/d) * 100
        speed = 0.3 #expirement with this
        self.arm.setExtendingArmPercentWithAuto(percent,speed)

    def end(self, interrupted: bool) -> None:
        self.arm.setGrabbingArmSpeedWithAuto(0)

    def isFinished(self) -> bool:
        return (percent + self.arm.tolerance >= self.arm.extendingArmEncoderPercent)