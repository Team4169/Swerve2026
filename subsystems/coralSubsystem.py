import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math
import phoenix6



class CoralSubsystem(commands2.SubsystemBase):
    

    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard
        
        self.liftingCoral = False
        self.liftCoralTimer = 0
        #* Intake motors
        self.coralEntranceMotor = rev.SparkMax(RobotConstants.coralIntakeMotor1ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.coralLiftMotor = rev.SparkMax(RobotConstants.coralIntakeMotor2ID, rev.SparkLowLevel.MotorType.kBrushless)

        self.liftConfig = rev.SparkMaxConfig()

        self.liftConfig.softLimit.forwardSoftLimit(50)
        self.liftConfig.softLimit.forwardSoftLimitEnabled()
        self.liftConfig.softLimit.reverseSoftLimit(0)
        self.liftConfig.softLimit.reverseSoftLimitEnabled()
        
        self.coralLiftMotor.configure(
            self.liftConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

    def runDepositCoral(self, speed: float):
        self.coralEntranceMotor.set(speed)

    def stopDepositCoral(self):
        self.coralEntranceMotor.set(0)

    def liftCoral(self, speed: float, time: float):
        print("starting lift coral")
        if time != 0:
            self.liftingCoral = True
            self.liftCoralTimer = time
        self.coralLiftMotor.set(speed)

    def stopLiftCoral(self):
        self.liftingCoral = False
        self.coralLiftMotor.set(0)

    # def lowCoral(self):
    #     self.coralEntranceMotor.set(0.5)
        

    # def runDepositCoral(self):
    #     self.speed = RobotConstants.coralMotorSpeed
    #     print("Running runDepositCoral")
        
    #     pass
    #     #TODO: deposit the coral using startCoral() and stopCoral()
    # def stopDepositCoral(self):
    #     self.speed = 0 
    #     print("Stopping runDepositCoral")

    # def execute(self):
    #     print("im coral and im executing")

    
    
