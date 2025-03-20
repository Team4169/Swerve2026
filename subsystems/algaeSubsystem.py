import wpilib, rev, commands2, wpilib.drive
from constants import RobotConstants

import phoenix6
class AlgaeSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard
        self.out = False
        self.canRun = True


        #* Intake motors
        self.algaeIntakeMotorL = rev.SparkMax(RobotConstants.algaeIntakeMotorLID, rev.SparkLowLevel.MotorType.kBrushless)
        self.algaeIntakeMotorR = rev.SparkMax(RobotConstants.algaeIntakeMotorRID, rev.SparkLowLevel.MotorType.kBrushless)
        self.algaeLiftMotor = rev.SparkMax(RobotConstants.algaeLiftMotorID, rev.SparkLowLevel.MotorType.kBrushless)
        self.liftConfig = rev.SparkMaxConfig()

        self.liftConfig.softLimit.reverseSoftLimit(0)
        self.liftConfig.softLimit.reverseSoftLimitEnabled(True)
        
        self.algaeLiftMotor.configure(
            self.liftConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

    def runAlgae(self, speed: float):
        self.algaeIntakeMotorL.set(speed)
        self.algaeIntakeMotorR.set(-speed)
    
    def stopAlgae(self):
        self.algaeIntakeMotorL.set(0)
        self.algaeIntakeMotorR.set(0)

    def runLiftAlgae(self, speed: float):
        if not self.canRun:
            return
        if speed >= 0:
            self.out = True
        if speed <= 0:
            self.out = False
        self.algaeLiftMotor.set(speed)

    def stopLiftAlgae(self):
        self.algaeLiftMotor.set(-.2 * 0.3)