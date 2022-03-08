import wpilib.drive
import ctre
import rev
from constants import constants
from networktables import NetworkTables
import navx

'''
D-pad left/right: Turn 90 deg left or right
X: Taunt
'''

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

        self.controller = wpilib.XboxController(0)
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
        self.direction = 1 # 1 or -1
        self.straightMode = False
        self.speed = [1, 1]
        self.turn = [False, False]
        self.humancontrol = True
        self.motor = [0, 0]

    def teleopPeriodic(self):
        self.output('Drive X', self.controller.getLeftX())
        self.output('Drive Y', self.controller.getLeftY())
        self.output('Gyro Yaw', self.gyro.getYaw())
        # self.output('Left Encoder', self.leftTalon.getSelectedSensorPosition())
        # self.output('Right Encoder', self.rightTalon.getSelectedSensorPosition())
        self.output('Lift Encoder', self.liftEncoder.getPosition())
        self.output('Rotate Encoder', self.rotateEncoder.getPosition())

        # if self.controller.getPOV() == 90:
        #     self.turnright90()

        if self.controller.getAButton():
            # Intake
            self.snowveyor.arcadeDrive(0.3, 0)
            pass
        elif self.controller.getBButton():
            pass
        elif self.controller.getYButton():
            # Outtake
            self.snowveyor.arcadeDrive(0.3, 0.3)
            pass
        elif self.controller.getXButton():
            pass


        if self.controller.getRightTriggerAxis() >= 0.2:
            self.liftArm.set(0.8)
        elif self.controller.getLeftTriggerAxis() >= 0.2:
            self.liftArm.set(-0.8)
        else:
            self.liftArm.set(0)

        if self.controller.getRightBumper():
            self.rotateArm.set(.2)
            self.output("rotate arm going","forward")
        elif self.controller.getLeftBumper():
            self.output("rotate arm going","back")
            self.rotateArm.set(-0.2)
        else:
            self.output("rotate arm going","none")

            self.rotateArm.set(0)

        self.straightMode = self.controller.getLeftBumperPressed()
        self.direction = -1 if self.controller.getRightBumperPressed() else 1


        if self.humancontrol:
            print('human' + str(self.motor[0]) + str(self.motor[1]))
            self.motor = [self.controller.getLeftY() * self.speed[0], self.controller.getLeftX() * self.speed[1]]
        else:
            print('robot' + str(self.motor[0]) + str(self.motor[1]))

        self.drive.arcadeDrive(self.motor[0], self.motor[1])

    def turnright90(self):
        self.humancontrol = False
        self.motor = [0, 0.5]

    def turnleft90(self):
        yaw = self.gyro.getYaw()
        if abs(self.gyro.getYaw() - yaw) < 0.01:
            self.speed = [1 + ((self.gyro.getYaw() - yaw)/90) ** 4, 1 - ((self.gyro.getYaw() - yaw)/90) ** 4]

if __name__ == "__main__":
  wpilib.run(MyRobot)