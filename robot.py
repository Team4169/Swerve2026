import wpilib.drive
import ctre
import rev
from constants import constants
from deadzone import addDeadzone
from networktables import NetworkTables
import navx

class MyRobot(wpilib.TimedRobot):
    def output(self, text, value):
      print(text + ': ' + str(value))
      self.sd.putValue(text, str(value))

    def robotInit(self):

        self.leftTalon = ctre.WPI_TalonSRX(constants["leftTalon"])
        self.leftVictor = ctre.WPI_VictorSPX(constants["leftVictor"])
        self.leftVictor.setInverted(True)
        self.leftTalon.setInverted(True)

        self.rightTalon = ctre.WPI_TalonSRX(constants["rightTalon"])
        self.rightVictor = ctre.WPI_VictorSPX(constants["rightVictor"])

        self.left = wpilib.SpeedControllerGroup(self.leftTalon, self.leftVictor)
        self.right = wpilib.SpeedControllerGroup(self.rightTalon, self.rightVictor)

        self.drive = wpilib.drive.DifferentialDrive(self.right, self.left)

        self.intake = ctre.WPI_VictorSPX(constants["intake"])
        self.outtake = ctre.WPI_VictorSPX(constants["outtake"])
        self.snowveyor = wpilib.drive.DifferentialDrive(self.intake, self.outtake)

        self.liftArm = rev.CANSparkMax(constants["liftArm"], rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.rotateArm = rev.CANSparkMax(constants["rotateArm"], rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.rotateEncoder = self.rotateArm.getEncoder()
        self.liftEncoder = self.liftArm.getEncoder(rev.SparkMaxRelativeEncoder.Type.kQuadrature)

        self.leftTalon.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        self.rightTalon.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)

        self.driverController = wpilib.XboxController(0)
        self.operatorController = wpilib.XboxController(1)
        
        self.timer = wpilib.Timer()
        self.sd = NetworkTables.getTable("SmartDashboard")
        self.gyro = navx.AHRS.create_i2c()


    def autnomousInit(self):
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        self.output('Time', self.timer.get())

    def teleopInit(self):
        print("Starting teleop...")
        self.humancontrol = True
        self.motor = [0, 0]
        self.intake = 0
        self.outtake = 0
        self.climbMode = False

    def teleopPeriodic(self):
        self.output('Drive X', self.driverController.getLeftX())
        self.output('Drive Y', self.driverController.getLeftY())
        self.output('Gyro Yaw', self.gyro.getYaw())
        # self.output('Left Encoder', self.leftTalon.getSelectedSensorPosition())
        # self.output('Right Encoder', self.rightTalon.getSelectedSensorPosition())
        self.output('Lift Encoder', self.liftEncoder.getPosition())
        self.output('Rotate Encoder', self.rotateEncoder.getPosition())

        self.straightMode = self.controller.getLeftBumperPressed()
        self.direction = -1 if self.controller.getRightBumperPressed() else 1

        lspeed = addDeadzone(self.driverController.getLeftTriggerAxis()) * self.direction
        rspeed = addDeadzone(self.driverController.getRightTriggerAxis()) * self.direction

        if self.operatorController.getStartButtonPressed():
            self.climbMode = not self.climbMode

        if self.climbMode:
            # TODO:
            # Put climb code here
            # MUST give drive train command AND return at end of function
            return


        if self.straightMode:
            whichbumper = (self.controller.getRightTriggerAxis() + self.controller.getLeftTriggerAxis())/2
            if self.driverController.getRightTriggerAxis() < 0.2:
                whichbumper = self.controller.getRightTriggerAxis()
            elif self.controller.getLeftTriggerAxis() < 0.2:
                whichbumper = self.controller.getLeftTriggerAxis()
            lspeed = whichbumper * self.direction
            rspeed = whichbumper * self.direction


        if self.driverController.getAButton():
            lspeed *= 0.5
            rspeed *= 0.5
        elif self.driverController.getBButton():
            lspeed *= 0.25
            rspeed *= 0.25
        elif self.driverController.getYButton():
            lspeed *= 0.1
            rspeed *= 0.1
        elif self.driverController.getXButton():
            pass

        if self.driverController.getRightBumper():
            lspeed *= -1
            rspeed *= -1

        if self.operatorController.getLeftTriggerAxis() > 0.2:
            self.snowveyor.arcadeDrive(1,0)

        elif self.operatorController.getRightTriggerAxis() > 0.2:
            self.snowveyor.arcadeDrive(1,1)

        elif self.operatorController.getLeftBumper():
            self.snowveyor.arcadeDrive(-1,0)

        elif self.operatorController.getRightBumper():
            self.snowveyor.arcadeDrive(-1,-1)


        if abs(self.gyro.getYaw() - self.yaw) > 80:
            self.humancontrol = True

        self.drive.arcadeDrive(lspeed, rspeed)


  
    def turnright90(self):
        self.yaw = self.gyro.getYaw()
        self.motor = [0, 0.5]
        self.humancontrol = False

    def turnleft90(self):
        self.yaw = self.gyro.getYaw()
        self.motor = [0, -0.5]
        self.humancontrol = False

if __name__ == "__main__":
  wpilib.run(MyRobot)