import wpilib, rev, commands2, wpilib.drive
from constants import RobotConstants
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


        self.out = False
        self.canRun = True

        #* Intake motors
        self.coralEntranceMotor = rev.SparkMax(RobotConstants.coralIntakeMotorID, rev.SparkLowLevel.MotorType.kBrushless)
        self.coralLiftMotor = rev.SparkMax(RobotConstants.coralLiftMotorID, rev.SparkLowLevel.MotorType.kBrushless)

        self.liftConfig = rev.SparkMaxConfig()

        self.liftConfig.softLimit.forwardSoftLimit(200)
        self.liftConfig.softLimit.forwardSoftLimitEnabled(True)
        self.liftConfig.softLimit.reverseSoftLimit(0)
        self.liftConfig.softLimit.reverseSoftLimitEnabled(True)
        
        self.coralLiftMotor.configure(
            self.liftConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        # self.depositConfig = rev.SparkMaxConfig()

        # self.depositConfig.softLimit.forwardSoftLimit(7)
        # self.depositConfig.softLimit.forwardSoftLimitEnabled(True)
        # self.depositConfig.softLimit.reverseSoftLimit(0)
        # self.depositConfig.softLimit.reverseSoftLimitEnabled(True)

        # self.coralEntranceMotor.configure(
        #     self.depositConfig,
        #     rev.SparkBase.ResetMode.kResetSafeParameters,
        #     rev.SparkBase.PersistMode.kPersistParameters
        # )

        self.myservo = wpilib.Servo(0)
        self.myservo.setAngle(0)

    def runDepositCoral(self, speed: float):
        if not self.canRun:
            return
        if speed <= 0:
            self.out = True
        if speed >= 0:
            self.out = False
        self.coralEntranceMotor.set(speed)

    def stopDepositCoral(self):
        self.coralEntranceMotor.set(0)


    
    # def liftCoral(self, speed: float, time: float):
    #     print("starting lift coral")
    #     if time != 0:
    #         self.liftingCoral = True
    #         self.liftCoralTimer = time
    #     self.coralLiftMotor.set(speed)

    def SimpleliftCoral(self, speed: float): 
        self.coralLiftMotor.set(speed)

    def stopLiftCoral(self):
        self.liftingCoral = False
        self.coralLiftMotor.set(0)

    def testServo(self):
        self.myservo.setAngle(0)
    def testServo2(self):
        self.myservo.setAngle(180)

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

    
    
