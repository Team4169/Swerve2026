#!/usr/bin/env python3
import typing
import wpilib
from wpimath.geometry import Rotation2d 
import commands2
import ctre
import math
import constants
from robotcontainer import RobotContainer
from commands.TeleopCommands.SwerveJoystickCmd import SwerveJoystickCmd
import ntcore
import robotpy_apriltag
import time
import numpy


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    AutonomousCommand: typing.Optional[commands2.Command] = None
    
    def robotInit(self) -> None:
        self.sd = wpilib.SmartDashboard
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        # wpilib.CameraServer().launch("vision.py:main")
        self.Container = RobotContainer()

        self.driverController = self.Container.driverController
        self.operatorController = self.Container.operatorController
        
        # self.drive = self.container.drive
        self.Swerve = self.Container.Swerve
        #~ LED commands and variables
        self.LEDserver = wpilib.I2C(wpilib.I2C.Port.kMXP, 100)
        self.previousLEDCommand = 0
    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""


    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        #~ will be needed for future use
        self.AutonomousCommand = self.Container.getAutonomousCommand()
        
        # self.output("ato com", self.autonomousCommand)
        #
        if self.AutonomousCommand:
            self.AutonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""


    def teleopInit(self) -> None:
        self.ds = wpilib.DriverStation

        # wpilib.CameraServer.launch()
        self.isRedAlliance = self.ds.getAlliance() == self.ds.Alliance.kRed
        self.sendLEDCommand(3, self.isRedAlliance)
        # self.drive.resetEncoders()
        self.moveRestriction = 1
        
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.AutonomousCommand:
            self.AutonomousCommand.cancel()

        # print("Starting teleop...")
        self.speed = 0

    def teleopPeriodic(self):
        #TODO put code in try and except functions, shown here https://robotpy.readthedocs.io/en/stable/guide/guidelines.html#don-t-die-during-the-competition
        # try:
        #     pass
        # except:
        #     if not self.ds.isFMSAttached():
        #         raise


        # print(wpilib.DriverStation.getAlliance())

        # self.sd.putNumber("gyroYaw", self.drive.gyro.getYaw())
        # self.sd.putNumber("gyroPitch", self.drive.gyro.getPitch())

        # # self.drive.encoderTicks.set(self.drive.)
        # self.sd.putNumber("left talon", self.leftTalon.getSelectedSensorPosition())
        # self.sd.putNumber("right talon", self.rightTalon.getSelectedSensorPosition())
        
        
        # #~ smartDashboard limit switch setting

        # self.sd.putBoolean("grabbingArmLimitSwitchClosed", self.arm.getGrabbingArmLimitSwitchClosedPressed())
        # self.sd.putBoolean("grabbingArmLimitSwitchOpen", self.arm.getGrabbingArmLimitSwitchOpenPressed())
        # self.sd.putBoolean("extendingArmLimitSwitchMin", self.arm.getExtendingArmLimitSwitchMinPressed())
        # self.sd.putBoolean("extendingArmLimitSwitchMax", self.arm.getExtendingArmLimitSwitchMaxPressed())
        # self.sd.putBoolean("rotatingArmLimitSwitchMax", self.arm.getRotatingArmLimitSwitchMaxPressed())
        # self.sd.putBoolean("rotatingArmLimitSwitchMin", self.arm.getRotatingArmLimitSwitchMinPressed())
    
        # self.sd.putNumber("rotatingArmEncoderDegrees", self.arm.rotatingArmEncoderDegrees)
        # self.sd.putNumber("extendingArmEncoderPercent", self.arm.extendingArmEncoderPercent)
        # self.sd.putNumber("grabbingArmEncoderDegrees", self.arm.grabbingArmEncoderDegrees)

        # self.sd.putNumber("grabbingArmEncoderDegrees", self.arm.grabbingArmEncoderDegrees)
        # self.sd.putNumber("extendingArmEncoderRevolutions", self.arm.extendingArmEncoder.getPosition())
        # self.sd.putNumber("rotatingArmEncoderRevolutions", self.arm.rotatingArmEncoder.getPosition())

    #todo: decide which controller this is on
    
    #^ start of driving code
        # self.leftX = addDeadzone(self.driverController.getLeftX() * self.moveRestriction) #* (self.sd.getNumber("moverestriction", constants.moveRestriction))) #/2 is to slow down the robot
        # self.leftY = addDeadzone(self.driverController.getLeftY() * self.moveRestriction) #* (self.sd.getNumber("moverestriction", constants.moveRestriction)))
        # self.rightX = addDeadzone(self.driverController.getRightX() * self.moveRestriction) #* (self.sd.getNumber("moverestriction", constants.moveRestriction)))
        
        # self.moving = self.leftX != 0 or self.leftY != 0 or self.rightX != 0
            
        # if self.driverController.getBButton():
        #     self.arm.setGrabbingArmSpeed(0.1)
        # elif self.driverController.getXButton():
        #     self.arm.setGrabbingArmSpeed(-0.1)
        # elif self.driverController.getYButton():
        #     self.arm.setGrabbingArmAngle(90, 0.09)
        # elif self.driverController.getAButton():
        #     self.arm.setGrabbingArmAngle(45, 0.09)
        # else:
        #     self.arm.setGrabbingArmSpeed(0)
            
        # self.flipped = 1
    #^ balancing with the A button
        
        # self.drive.driveMecanum( -self.leftY * self.flipped, self.leftX * self.flipped, self.rightX, Rotation2d(self.gyroRad)) #self.gyroRad

       
        # * arm control  
        #Todo: make sure the "foreward" is positive
    
    #^: arm functions
        # self.arm.setRotatingArmSpeedWithAuto(self.operatorController.getLeftY())
        
        # if self.operatorController.getLeftY() > constants.deadzone or self.operatorController.getLeftY() < -constants.deadzone:
        #     self.sendLEDCommand(1, self.isRedAlliance)
        # elif self.operatorController.getRightY() < constants.deadzone and self.operatorController.getRightY() > -constants.deadzone and self.operatorController.getLeftY() < constants.deadzone and self.operatorController.getLeftY() > -constants.deadzone:
        #     self.sendLEDCommand(3, self.isRedAlliance)
        # else:
        #     self.sendLEDCommand(2, self.isRedAlliance)
        pass
    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def sendLEDCommand(self, command, isRedAlliance = None):
            # send the specified command to the LEDserver
            team_command = isRedAlliance
            if not isRedAlliance:
                team_command = command + 3
            if self.previousLEDCommand != team_command:
                self.previousLEDCommand = team_command
                if self.LEDserver.writeBulk(memoryview(bytes([team_command]))):
                    print("Got an error sending command ", team_command)
                    return True
                else:
                    print("Success sending command ", team_command)
                    return False
                


if __name__ == "__main__":
    wpilib.run(MyRobot)
# #!/usr/bin/env python3
