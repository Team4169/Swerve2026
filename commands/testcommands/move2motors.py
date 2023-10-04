import commands2
from constants import RobotConstants
import rev
import time
class move2motors(commands2.CommandBase):
    def __init__(self) -> None:
        super().__init__()

    def initialize(self):
        self.drivingMotor = rev.CANSparkMax(RobotConstants.drivingMotorID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(RobotConstants.turningMotorID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        # self.drivingMotor.setInverted(drivingMotorReversed)
        # self.turningMotor.setInverted(turningMotorReversed)

        self.drivingEncoder = self.drivingMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        self.drivingMotor.set(.25)
        self.turningMotor.set(.25)

        self.startTime = time.time()
        self.runTime = 5


    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        self.drivingMotor.set(0)
        self.turningMotor.set(0)

    def isFinished(self) -> bool:
        self.currentTime = time.time()
        if self.currentTime - self.startingTime > self.runTime:
            return True

        return False