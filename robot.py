import wpilib.drive
import ctre
from constants import constants
from networktables import NetworkTables
import navx

class MyRobot(wpilib.TimedRobot):
    def output(self, text, value):
      print(text + ': ' + str(value))
      self.sd.putValue(text, str(value))

    def robotInit(self):
        self.front_left_motor = ctre.WPI_TalonSRX(constants["frontLeftPort"])
        self.rear_left_motor = ctre.WPI_TalonSRX(constants["rearLeftPort"])
        self.front_left_motor.setInverted(True)
        self.rear_left_motor.setInverted(True)
        self.left = wpilib.SpeedControllerGroup(self.front_left_motor, self.rear_left_motor)
      
        self.front_right_motor = ctre.WPI_TalonSRX(constants["frontRightPort"])
        self.rear_right_motor = ctre.WPI_TalonSRX(constants["rearRightPort"])
        self.right = wpilib.SpeedControllerGroup(self.front_right_motor, self.rear_right_motor)

        self.drive = wpilib.drive.DifferentialDrive(self.right, self.left)

        self.intake = ctre.WPI_VictorSPX(constants["intake"])
        self.outtake = ctre.WPI_VictorSPX(constants["outtake"])
        self.snowveyor = wpilib.drive.DifferentialDrive(self.intake, self.outtake)
      
        self.driverController = wpilib.XboxController(0)
        self.snowveyorController = wpilib.XboxController(1)
      
        self.timer = wpilib.Timer()
        self.sd = NetworkTables.getTable("SmartDashboard")
        self.gyro = navx.AHRS(wpilib.SerialPort.Port.kUSB1)
        self.front_left_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        self.front_right_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)

    def autnomousInit(self):
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        self.output('Time', self.timer.get())

    def teleopInit(self):
        print("Starting teleop...")
        self.speed = [1, 1]
        self.humancontrol = True
        self.motor = [0, 0]
        self.intake = 0
        self.outtake = 0

    def teleopPeriodic(self):
        self.output('Drive X', self.driverController.getLeftX())
        self.output('Drive Y', self.driverController.getLeftY())
        self.output('Gyro Yaw', self.gyro.getYaw())
        self.output('Left Encoder', self.front_left_motor.getSelectedSensorPosition())
        self.output('Right Encoder', self.front_right_motor.getSelectedSensorPosition())

        # driverController
        if self.driverController.getPOV() == 90:
            self.turnright90()

        if self.driverController.getPOV() == 270:
            self.turnleft90()

        if self.driverController.getYButton():
            self.speed = [1, 1]
        elif self.driverController.getBButton():
            self.speed = [0.8, 0.8]
        elif self.driverController.getAButton():
            self.speed = [0.6, 0.6]
        elif self.driverController.getXButton():
            self.speed = [0.4, 0.4]

        if self.driverController.getLeftBumperPressed():
            self.speed[1] = 0

        if self.driverController.getRightBumperPressed():
            self.speed = [-self.speed[0], -self.speed[1]]

        if self.humancontrol:
            self.motor = [self.driverController.getLeftY() * self.speed[0], self.driverController.getLeftX() * self.speed[1]]
        else:
            print(str(self.gyro.getYaw()) + ' ' + str(self.yaw) + ' ' + str(self.gyro.getYaw() - self.yaw))
            if abs(self.gyro.getYaw() - self.yaw) > 80:
                self.humancontrol = True

        self.drive.arcadeDrive(self.motor[0], self.motor[1])

        # snowveyorController
        if self.snowveyorController.getRightBumper():
          self.intake = 1
        else:
          self.intake = 0

        if self.snowveyorController.getLeftBumper():
          self.outtake = 1
        else:
          self.outtake = 0

        self.snowveyor.arcadeDrive(self.intake, self.outtake)
  
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