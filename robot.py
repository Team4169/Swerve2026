#!/usr/bin/env python3

import typing
import wpilib
import commands2

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def output(self, text, value):
      print(text + ': ' + str(value))
      self.container.drive.sd.putValue(text, str(value))

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        print("Starting teleop...")
        self.speed = [1, 1]
        self.humancontrol = True
        self.motor = [0, 0]
        self.intakeSpeed = 0
        self.outtakeSpeed = 0

    def teleopPeriodic(self) -> None:
        # self.output('Drive X', self.container.driverController.getLeftX())
        # self.output('Drive Y', self.container.driverController.getLeftY())
        # self.output('Gyro Yaw', self.container.drive.gyro.getYaw())

        # driveController
        if self.container.driverController.getPOV() == 90:
            self.turnright90()

        if self.container.driverController.getPOV() == 270:
            self.turnleft90()

        if self.container.driverController.getYButton():
            self.speed = [1, 1]
        elif self.container.driverController.getBButton():
            self.speed = [0.8, 0.8]
        elif self.container.driverController.getAButton():
            self.speed = [0.6, 0.6]
        elif self.container.driverController.getXButton():
            self.speed = [0.4, 0.4]

        if self.container.driverController.getLeftBumperPressed():
            self.speed[1] = 0

        if self.container.driverController.getRightBumperPressed():
            self.speed = [-self.speed[0], -self.speed[1]]

        if self.humancontrol:
            self.motor = [self.container.driverController.getLeftY() * self.speed[0], self.container.driverController.getLeftX() * self.speed[1]]
        else:
            if abs(self.container.drive.gyro.getYaw() - self.yaw) > 80:
                self.humancontrol = True

        self.container.drive.arcadeDrive(self.motor[0], self.motor[1])

        # snowveyorController
        if self.container.snowveyorController.getRightBumper():
          self.intakeSpeed = 1
        else:
          self.intakeSpeed = 0

        if self.container.snowveyorController.getLeftBumper():
          self.outtakeSpeed = 1
        else:
          self.outtakeSpeed = 0

        self.container.drive.snowveyor.tankDrive(self.intakeSpeed, self.outtakeSpeed)

    def turnright90(self):
        self.yaw = self.container.drive.gyro.getYaw()
        self.motor = [0, 0.5]
        self.humancontrol = False

    def turnleft90(self):
        self.yaw = self.container.drive.gyro.getYaw()
        self.motor = [0, -0.5]
        self.humancontrol = False

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
