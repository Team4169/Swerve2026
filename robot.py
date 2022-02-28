import wpilib.drive
import ctre
from constants import constants
from networktables import NetworkTables
import navx

'''
D-pad left/right: Turn 90 deg left or right
X: Taunt
'''

class MyRobot(wpilib.TimedRobot):
    def output(self, text, value):
      # print(text + ': ' + str(value))
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

        self.controller = wpilib.XboxController(0)
        self.timer = wpilib.Timer()
        self.sd = NetworkTables.getTable("SmartDashboard")
        self.gyro = navx.AHRS.create_i2c()
        self.front_left_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        self.front_right_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)

    def autnomousInit(self):
        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        self.output('Time', self.timer.get())

    def teleopInit(self):
        print("Starting teleop...")
        self.mode = [False, False] # ['straight', 'backward']
        self.speed =  [1, 1]
        self.turn = [False, False]

    def teleopPeriodic(self):
        self.output('Drive X', self.controller.getLeftX())
        self.output('Drive Y', self.controller.getLeftY())
        self.output('Gyro Yaw', self.gyro.getYaw())
        self.output('Left Encoder', self.front_left_motor.getSelectedSensorPosition())
        self.output('Right Encoder', self.front_right_motor.getSelectedSensorPosition())

        self.turn[0] = self.controller.getPOV() == 90
        self.turn[1] = self.controller.getPOV() == 270

        if self.controller.getAButton():
            self.speed = [0.5, 0.5]
        elif self.controller.getBButton():
            self.speed = [0.25, 0.25]
        elif self.controller.getYButton():
            self.speed = [0.1, 0.1]
        else:
            self.speed = [0.75, 0.75]

        self.mode = not self.mode if self.controller.getLeftBumperPressed() else self.mode # straight mode
        self.mode = not self.mode if self.controller.getRightBumperPressed() else self.mode # backward mode
        print(f"Speed is {self.speed}")
        self.drive.arcadeDrive(self.controller.getLeftY() * self.speed[0], self.controller.getLeftX() * self.speed[1])

    def turnright90(self):
        yaw = self.gyro.getYaw()
        if abs(self.gyro.getYaw() - yaw) < 0.01:
            self.speed = [1 - ((self.gyro.getYaw() - yaw)/90) ** 4, 1 + ((self.gyro.getYaw() - yaw)/90) ** 4]

    def turnleft90(self):
        yaw = self.gyro.getYaw()
        if abs(self.gyro.getYaw() - yaw) < 0.01:
            self.speed = [1 + ((self.gyro.getYaw() - yaw)/90) ** 4, 1 - ((self.gyro.getYaw() - yaw)/90) ** 4]

if __name__ == "__main__":
  wpilib.run(MyRobot)