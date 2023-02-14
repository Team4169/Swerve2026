#!/usr/bin/env python3

import typing
import wpilib
from wpimath.geometry import Rotation2d 
import commands2
import ctre
import math
import constants
from robotcontainer import RobotContainer
from deadzone import addDeadzone
import ntcore



class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def output(self, text, value):
        pass
      # print(text + ': ' + str(value))
      # self.container.drive.sd.putValue(text, str(value))

    def robotInit(self) -> None:
        
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        # wpilib.CameraServer().launch("vision.py:main")
        self.container = RobotContainer()

        self.driverController = self.container.driverController
        # self.operatorController = self.container.operatorController

        self.leftTalon = self.container.leftTalon
        self.leftTalon2 = self.container.leftTalon2
        self.rightTalon = self.container.rightTalon
        self.rightTalon2 = self.container.rightTalon2

        self.mech = wpilib.drive.MecanumDrive(
            self.leftTalon,
            self.leftTalon2,
            self.rightTalon,
            self.rightTalon2,
        )
        # self.neoMotor = self.container.neoMotor

        #self.liftArm = self.container.liftArm
        #self.rotateArm = self.container.rotateArm

        # self.rotateEncoder = self.container.rotateEncoder
        # self.liftEncoder = self.container.liftEncoder

        # self.liftArmUpLimitSwitch = self.container.liftArmUpLimitSwitch
        # self.liftArmDownLimitSwitch = self.container.liftArmDownLimitSwitch
        #self.rotateArmBackLimitSwitch = self.container.rotateArmBackLimitSwitch
        #self.rotateArmRobotLimitSwitch = self.container.rotateArmRobotLimitSwitch

        # self.intake = self.container.intake
        # self.outtake = self.container.outtake
        # self.snowveyor = self.container.snowveyor

        self.drive = self.container.drive



    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()
        self.output("ato com", self.autonomousCommand)
        #
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        #write auto code here

    def teleopInit(self) -> None:

        self.container.drive.resetEncoders()
        self.container.drive.gyro.reset()

        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # print("Starting teleop...")
        self.humancontrol = True
        self.speed = 0
        self.intake = 0
        self.outtake = 0
        self.climbMode = False
        self.direction = 0

 

    def teleopPeriodic(self):
        #self.neoMotor.set(self.driverController.getRightTriggerAxis()/4)  # sets neo motor running at power = .1 out of 1
        self.container.drive.gyroOut.set(self.container.drive.gyro.getYaw())
        self.container.drive.gyroPitchOut.set(self.container.drive.gyro.getPitch())

        self.container.drive.encoderRightOut.set(self.rightTalon.getSelectedSensorPosition())
        self.container.drive.encoderLeftOut.set(self.leftTalon.getSelectedSensorPosition())

        # self.output("current brake mode", self.container.climb.rotateArm.getIdleMode())
        # self.output("liftencoder value new", self.container.climb.liftEncoder.getPosition())
        # self.output("newdriveencodervalueleft", self.container.drive.leftTalon.getSelectedSensorPosition())
        # self.output("newdriveencodervalueright", self.container.drive.rightTalon.getSelectedSensorPosition())
        # self.output("climb mode",self.climbMode)

        if self.driverController.getLeftBumper():
            self.output("straight mode", True)
            self.direction = 0
        else:
            self.output("straight mode", False)
            self.direction = self.driverController.getLeftX()
        self.leftX = addDeadzone(self.driverController.getLeftX())
        self.leftY = addDeadzone(self.driverController.getLeftY())
        self.rightX = addDeadzone(self.driverController.getRightX())
        #There are 2 different ways of programming mecanum, this is the from the first
        #note the direction of the motors on the right must be reversed 
        
        
        # self.speed = addDeadzone(self.driverController.getLeftY()) * -1 # TODO: Clean up
        # self.mag = math.sqrt(self.leftX**2 + self.leftY**2)
        # self.angle = math.atan2(self.leftY, self.leftX)

        # self.frontLeftBackRight = math.sin(self.angle+ .25*math.pi) * self.mag
        # self.frontRightBackLeft = math.sin(self.angle - .25 * math.pi) * self.mag
        # #code that sets the motors to their correct speeds
        # self.leftTalon.set(self.frontLeftBackRight)
        # self.leftTalon2.set(self.frontRightBackLeft)
        # self.rightTalon.set(self.frontRightBackLeft)
        # self.rightTalon2.set(self.frontLeftBackRight)

    #this is from the second
    #note the direction of the motors on the right must be reversed
        # print(self.container.drive.gyro.getYaw())
        self.gyroRad = self.container.drive.gyro.getYaw() * (math.pi/180)
        self.rotX = self.leftX * math.cos(-self.gyroRad) - self.leftY * math.sin(-self.gyroRad)
        self.rotY = self.leftX * math.sin(-self.gyroRad) + self.leftY * math.cos(-self.gyroRad)

        self.denom = max(abs(self.leftY) + abs(self.leftX) + abs(self.rightX), 1);

        self.frontLeftMotor = (self.rotY + self.rotX + self.rightX) / self.denom
        self.backLeftMotor = (self.rotY - self.rotX + self.rightX) / self.denom
        self.frontRightMotor = (self.rotY - self.rotX - self.rightX) / self.denom
        self.backRightMotor = (self.rotY + self.rotX - self.rightX) / self.denom

        # self.leftTalon.set(self.frontLeftMotor)
        # self.leftTalon2.set(self.backLeftMotor)
        # self.rightTalon.set(self.frontRightMotor)
        # self.rightTalon2.set(self.backRightMotor)
        
            

        #XXX: This is test for each individual motor
        # if self.driverController.getAButton():
        #     self.leftTalon2.set(0.5)
        # else:
        #     self.leftTalon2.set(0)
        
        # if self.driverController.getBButton():
        #     self.rightTalon2.set(0.5)
        # else:
        #     self.rightTalon2.set(0)

        # if self.driverController.getYButton():
        #     self.rightTalon.set(0.5)
        # else:
        #     self.rightTalon.set(0)
        
        # if self.driverController.getXButton():
        #     self.leftTalon.set(0.5)
        # else:

        #     self.leftTalon.set(0)
        self.container.drive.balanceSensitivitySub.get()
        
        if not self.driverController.getAButton():
            self.mech.driveCartesian( -self.leftY, self.leftX, self.rightX, Rotation2d(self.gyroRad)) #self.gyroRad
        else:
            self.pitchAngle = self.container.drive.gyro.getPitch()
            self.speed = constants.maxBalanceSpeed*2/(1 + math.e**(-constants.balanceSensitivity*(self.pitchAngle/constants.maxBalanceAngle)))-constants.maxBalanceSpeed #min(max(-abs(self.pitchAngle) + , 0), 1)
            # maybe make it drive cartesian so that the robot can balance while sideways
            self.leftTalon.set(self.speed)
            self.rightTalon.set(self.speed)
            self.leftTalon2.set(self.speed)
            self.rightTalon2.set(self.speed)
            

        # addDeadzone(


        # if self.operatorController.getLeftTriggerAxis() > 0.2:
        #     self.snowveyor.tankDrive(1,0)
        #
        # elif self.operatorController.getRightTriggerAxis() > 0.2:
        #     self.snowveyor.tankDrive(1,-1)
        #
        # elif self.operatorController.getLeftBumper():
        #     self.snowveyor.tankDrive(-1,0)
        #
        # elif self.operatorController.getRightBumper():
        #     self.snowveyor.tankDrive(-1,1)


        #self.drive.arcadeDrive(self.speed, self.direction)


    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
# #!/usr/bin/env python3
# """
#     This is a demo program showing how to use Mecanum control with the
#     MecanumDrive class.
# """
# import ctre
# import wpilib
# from wpilib.drive import MecanumDrive


# class MyRobot(wpilib.TimedRobot):
#     # Channels on the roboRIO that the motor controllers are plugged in to
#     frontLeftChannel = 3
#     rearLeftChannel = 7
#     frontRightChannel = 9
#     rearRightChannel = 4

#     # The channel on the driver station that the joystick is connected to
#     joystickChannel = 0

#     def robotInit(self):
#         """Robot initialization function"""
#         self.frontLeftMotor = ctre.WPI_TalonSRX(self.frontLeftChannel)
#         self.rearLeftMotor = ctre.WPI_TalonSRX(self.rearLeftChannel)
#         self.frontRightMotor = ctre.WPI_TalonSRX(self.frontRightChannel)
#         self.rearRightMotor = ctre.WPI_TalonSRX(self.rearRightChannel)

#         # invert the left side motors
#         self.frontRightMotor.setInverted(True)
#         self.frontLeftMotor.setInverted(True)

#         # you may need to change or remove this to match your robot
#         # self.rearRightMotor.setInverted(True)

#         self.drive = MecanumDrive(
#             self.frontLeftMotor,
#             self.rearLeftMotor,
#             self.frontRightMotor,
#             self.rearRightMotor,
#         )
#         # Define the Xbox Controller.
#         self.stick = wpilib.XboxController(self.joystickChannel)

#     def teleopInit(self):
#         self.drive.setSafetyEnabled(True)

#     def teleopPeriodic(self):
#         """Runs the motors with Mecanum drive."""
#         # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
#         # This sample does not use field-oriented drive, so the gyro input is set to zero.
#         # This Stick configuration is created by K.E. on our team.  Left stick Y axis is speed, Left Stick X axis is strafe, and Right Stick Y axis is turn.
#         self.drive.driveCartesian(
#             self.stick.getLeftX(),
#             self.stick.getLeftY(),
#             self.stick.getRightY(),
#         )

#         """Alternatively, to match the driver station enumeration, you may use  ---> self.drive.driveCartesian(
#             self.stick.getRawAxis(1), self.stick.getRawAxis(3), self.stick.getRawAxis(2), 0
#         )"""


# if __name__ == "__main__":
#     wpilib.run(MyRobot)