import wpilib, rev, commands2
import wpilib.drive
from constants import RobotConstants
import math

class ClimbingSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard

        #* Climbing motor
        self.ClimbingMotor1 = rev.SparkMax(RobotConstants.climbingMotorFrontID, rev.SparkLowLevel.MotorType.kBrushless)
        self.ClimbingMotor2 = rev.SparkMax(RobotConstants.climbingMotorBackID, rev.SparkLowLevel.MotorType.kBrushless)
    
        # self.climbingConfig = rev.SparkMaxConfig()

        # self.climbingConfig.softLimit.forwardSoftLimit(math.pi/2)
        # self.climbingConfig.softLimit.forwardSoftLimitEnabled(True)
        # self.climbingConfig.softLimit.reverseSoftLimit(-math.pi/2)
        # self.climbingConfig.softLimit.reverseSoftLimitEnabled(True)

        # self.ClimbingMotor1.configure(
        #     self.d  Config,
        #     rev.SparkBase.ResetMode.kResetSafeParameters,
        #     rev.SparkBase.PersistMode.kPersistParameters
        # )

    def runClimbingMotors(self, speed: float):
        self.ClimbingMotor1.set(speed)
        self.ClimbingMotor2.set(speed)

    def stopClimbingMotors(self):
        self.ClimbingMotor1.set(0)
        self.ClimbingMotor2.set(0)
