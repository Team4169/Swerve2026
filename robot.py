import wpilib.drive
import ctre
from constants import constants
from networktables import NetworkTables
import navx

'''
Left bumper: Lock straight
Right bumper: Lock reverse
Left and right bumpers: Lock straight backwards
D-pad left/right: Turn 90 deg left or right
X: Taunt
'''

class MyRobot(wpilib.TimedRobot):
    def output(self, text, value):
      print(text + ': ' + value)   
      self.sd.putValue(text, value)
      
    def robotInit(self):
        self.front_left_motor = ctre.WPI_TalonSRX(constants["frontLeftPort"])
        self.rear_left_motor = ctre.WPI_VictorSPX(constants["rearLeftPort"])
        self.rear_left_motor.setInverted(True)
        self.left = wpilib.SpeedControllerGroup(self.front_left_motor, self.rear_left_motor)

        self.front_right_motor = ctre.WPI_TalonSRX(constants["frontRightPort"])
        self.rear_right_motor = ctre.WPI_VictorSPX(constants["rearRightPort"])
        self.rear_right_motor.setInverted(True)
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
        self.output(self, 'Time', self.timer.get())

    def teleopInit(self):
        print("Starting teleop...")
        self.mode = 'individual'
        self.speed =  [1, 1]
  
    def teleopPeriodic(self):
        self.output(self, 'Drive X', self.controller.getX(self.controller.Hand.kLeftHand))
        self.output(self, 'Drive Y', self.controller.getY(self.controller.Hand.kLeftHand))
        self.output(self, 'Gyro Yaw', self.gyro.getYaw())
        self.output(self, 'Left Encoder', self.front_left_motor.getSelectedSonsorPosition())
        self.output(self, 'Right Encoder', self.front_right_motor.getSelectedSensorPosition())

        speed = 1
        # Slow mode
        if self.controller.getAButton():
            speed = 0.5
        elif self.controller.getBButton():
            speed = 0.25
        elif self.controller.getYButton():
            speed = 0.1
        #Reverse
        if self.controller.getRightBumperPressed():
            speed *= -1
        
        self.drive.arcadeDrive(self.controller.getX(self.controller.Hand.kLeftHand) * speed, self.controller.getY(self.controller.Hand.kLeftHand) * speed)

    def turnleft90(self):
        yaw = self.gyro.getYaw()
        while abs(self.gyro.getYaw() - yaw) < 0.01:
            self.speed = [1 - ((self.gyro.getYaw() - yaw)/90) ** 4, 1 + ((self.gyro.getYaw() - yaw)/90) ** 4]

    def turnleft90(self):
        yaw = self.gyro.getYaw()
        while abs(self.gyro.getYaw() - yaw) < 0.01:
          self.speed = [1 + ((self.gyro.getYaw() - yaw)/90) ** 4, 1 - ((self.gyro.getYaw() - yaw)/90) ** 4]

if __name__ == "__main__":
  wpilib.run(MyRobot)