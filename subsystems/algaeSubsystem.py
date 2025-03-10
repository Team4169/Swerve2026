import wpilib, rev, commands2, wpilib.drive
from constants import RobotConstants

class AlgaeSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard


        #* Intake motors
        self.algaeIntakeMotor1 = rev.SparkMax(RobotConstants.algaeIntakeMotor1ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.algaeIntakeMotor2 = rev.SparkMax(RobotConstants.algaeIntakeMotor2ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.algaeLiftMotor = rev.SparkMax(RobotConstants.liftMotorID, rev.SparkLowLevel.MotorType.kBrushless)

        self.liftConfig = rev.SparkMaxConfig()

        self.liftConfig.softLimit.forwardSoftLimit(50)
        self.liftConfig.softLimit.forwardSoftLimitEnabled(True)
        
        self.algaeLiftMotor.configure(
            self.liftConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

    def runAlgae(self, speed: float):
        self.algaeIntakeMotor1.set(speed)
        self.algaeIntakeMotor2.set(speed)
    
    def stopAlgae(self):
        self.algaeIntakeMotor1.set(0)
        self.algaeIntakeMotor2.set(0)

    def runLiftAlgae(self, speed: float):
        self.algaeLiftMotor.set(speed)

    def stopLiftAlgae(self):
        self.algaeLiftMotor.set(0)