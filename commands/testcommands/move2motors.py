import commands2
from constants import RobotConstants
import rev
import time

from subsystems.swervesubsystem import SwerveSubsystem

class move2motors(commands2.CommandBase):
    def __init__(self, swerve: SwerveSubsystem) -> None:
        super().__init__()

        self.swerve = swerve

    def initialize(self):
        self.drivingMotor = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(11, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        
        # self.drivingMotor.setInverted(drivingMotorReversed)
        # self.turningMotor.setInverted(turningMotorReversed)
        
        self.drivingMotor.set(0.1)
        self.turningMotor.set(0.1)
        self.drivingEncoder = self.drivingMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()
    
        self.startTime = time.time()
        self.runTime = 3



    def execute(self) -> None:
        self.swerve.sd.putString("Motor Position", str(self.drivingEncoder.getVelocity()))
        print("This code is being run. whoooo!")

        

    def end(self, interrupted: bool) -> None:
        self.drivingMotor.set(0)
        self.turningMotor.set(0)

    def isFinished(self) -> bool:
        self.currentTime = time.time()
        self.swerve.sd.putString("Delta time", str(self.currentTime - self.startTime))
        if self.currentTime - self.startTime > self.runTime:
            print(9)
            return True

        return False