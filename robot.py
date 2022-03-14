#!/usr/bin/env python3

import typing
import wpilib, wpilib.drive
import commands2
import ctre, navx, rev

import constants
from robotcontainer import RobotContainer

from networktables import NetworkTables
from deadzone import addDeadzone




class MyRobot(wpilib.TimedRobot):

    autonomousCommand: typing.Optional[commands2.Command] = None

    def output(self, text, value):
      print(text + ': ' + str(value))
      self.container.driveSystem.sd.putValue(text, str(value))


    def robotInit(self):

        self.leftTalon = ctre.WPI_TalonSRX(constants.leftTalon)
        self.leftVictor = ctre.WPI_VictorSPX(constants.leftVictor)
        self.leftVictor.setInverted(True)
        self.leftTalon.setInverted(True)

        self.rightTalon = ctre.WPI_TalonSRX(constants.rightTalon)
        self.rightVictor = ctre.WPI_VictorSPX(constants.rightVictor)

        self.left = wpilib.SpeedControllerGroup(self.leftTalon, self.leftVictor)
        self.right = wpilib.SpeedControllerGroup(self.rightTalon, self.rightVictor)

        self.drive = wpilib.drive.DifferentialDrive(self.right, self.left)

        self.intake = ctre.WPI_VictorSPX(constants.intake)
        self.outtake = ctre.WPI_VictorSPX(constants.outtake)
        self.snowveyor = wpilib.drive.DifferentialDrive(self.intake, self.outtake)

        
        self.liftArm = rev.CANSparkMax(constants.liftArm, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.rotateArm = rev.CANSparkMax(constants.rotateArm, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.liftArm.setIdleMode(self.liftArm.IdleMode(1))
        self.rotateArm.setIdleMode(self.rotateArm.IdleMode(1))

        self.liftArm.setInverted(True)


        self.liftEncoder = self.liftArm.getEncoder(rev.SparkMaxRelativeEncoder.Type.kQuadrature)
        self.rotateEncoder = self.rotateArm.getEncoder()
        

        self.liftArmUpLimitSwitch = wpilib.DigitalInput(0)
        self.liftArmDownLimitSwitch = wpilib.DigitalInput(constants.liftArmDownLimitSwitch)

        self.rotateArmRobotLimitSwitch = wpilib.DigitalInput(constants.rotateArmRobotLimitSwitch)
        self.rotateArmBackLimitSwitch = wpilib.DigitalInput(2)

        self.yaw = 0


        self.leftTalon.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        self.rightTalon.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        

        self.driverController = wpilib.XboxController(0)
        self.operatorController = wpilib.XboxController(1)

        self.timer = wpilib.Timer()
        self.sd = NetworkTables.getTable("SmartDashboard")
        self.gyro = navx.AHRS.create_i2c()
        
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer(driverController=self.driverController,
                                        operatorController=self.operatorController,
                                        drive=self.drive,
                                        snowveyor=self.snowveyor,
                                        liftArm=self.liftArm,
                                        rotateArm=self.rotateArm,
                                        liftEncoder=self.liftEncoder,
                                        rotateEncoder=self.rotateEncoder,
                                        liftArmUpLimitSwitch=self.liftArmUpLimitSwitch,
                                        rotateArmBackLimitSwitch=self.rotateArmBackLimitSwitch,
                                        liftArmDownLimitSwitch= self.liftArmDownLimitSwitch,
                                        rotateArmRobotLimitSwitch=self.rotateArmRobotLimitSwitch
                                        )




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
        self.humancontrol = True

        self.lspeed = 0
        self.rspeed = 0
        self.intake = 0
        self.outtake = 0
        self.climbMode = False



    def teleopPeriodic(self):
        self.output("limit switch", self.liftArmUpLimitSwitch.get())
        self.output("limit switch 2", self.rotateArmBackLimitSwitch.get())
        # self.output('Drive X', self.driverController.getLeftX())
        # self.output('Drive Y', self.driverController.getLeftY())
        # self.output('Gyro Yaw', self.gyro.getYaw())
        # # self.output('Left Encoder', self.leftTalon.getSelectedSensorPosition())
        # # self.output('Right Encoder', self.rightTalon.getSelectedSensorPosition())
        # self.output('Lift Encoder', self.liftEncoder.getPosition())
        # self.output('Rotate Encoder', self.rotateEncoder.getPosition())

        self.straightMode = self.driverController.getLeftBumperPressed()
        self.direction = -1 if self.driverController.getRightBumperPressed() else 1


        self.lspeed = addDeadzone(self.driverController.getLeftTriggerAxis()) * self.direction
        self.rspeed = addDeadzone(self.driverController.getRightTriggerAxis()) * self.direction


        if self.operatorController.getStartButtonPressed():
            self.climbMode = not self.climbMode

        if self.climbMode:
          
            if self.operatorController.getYButton() and self.liftArmUpLimitSwitch.get() != constants.liftArmUpLimitSwitchPressedValue:
                self.liftArm.set(0.6)
            elif self.operatorController.getAButton(): # and self.liftArmUpLimitSwitch.get() != constants.liftArmUpLimitSwitchPressedValue:

                self.liftArm.set(-0.6)
            else:
                self.liftArm.set(0)
                # pass

            if self.operatorController.getXButton(): # and self.rotateArmBackLimitSwitch.get() != constants.rotateArmBackLimitSwitchPressedValue:
                self.rotateArm.set(-0.1)
            elif self.operatorController.getBButton() and self.rotateArmBackLimitSwitch.get() != constants.rotateArmBackLimitSwitchPressedValue:
                self.rotateArm.set(0.1)
            else:
                self.rotateArm.set(0)

            dir = self.operatorController.getPOV()
            if 225 < dir <= 315:
                self.lspeed = 0
                self.rspeed = 0.2
            elif 135 < dir <= 225:
                self.lspeed = -0.2
                self.rspeed = -0.2
            elif 45 < dir <= 135:
                self.lspeed = 0.2
                self.rspeed = 0
            elif dir <= 45:
                self.lspeed = 0.2
                self.rspeed = 0.2
            else:
                self.lspeed = 0
                self.rspeed = 0
            self.drive.tankDrive(self.lspeed, self.rspeed)

            return


        if self.straightMode:
            whichbumper = (self.lspeed + self.rspeed) / 2
            if abs(self.lspeed) < 0.2:
                whichbumper = self.rspeed
            elif abs(self.rspeed) < 0.2:
                whichbumper = self.lspeed

            self.lspeed = whichbumper
            self.rspeed = whichbumper

        if self.driverController.getAButton():
            self.lspeed *= 0.5
            self.rspeed *= 0.5
        elif self.driverController.getBButton():
            self.lspeed *= 0.25
            self.rspeed *= 0.25
        elif self.driverController.getYButton():
            self.lspeed *= 0.1
            self.rspeed *= 0.1
        elif self.driverController.getXButton():
            pass

        if self.driverController.getRightBumper():
            self.lspeed *= -1
            self.rspeed *= -1

        if self.operatorController.getLeftTriggerAxis() > 0.2:
            self.snowveyor.tankDrive(1,0)

        elif self.operatorController.getRightTriggerAxis() > 0.2:
            self.snowveyor.tankDrive(1,-1)

        elif self.operatorController.getLeftBumper():
            self.snowveyor.tankDrive(-1,0)

        elif self.operatorController.getRightBumper():
            self.snowveyor.tankDrive(-1,-1)


        if abs(self.gyro.getYaw() - self.yaw) > 80:
            self.humancontrol = True


        self.output('self.lspeed', self.lspeed)
        self.output('self.rspeed', self.rspeed)

        if abs(self.gyro.getYaw() - self.yaw) > 80:
            self.humancontrol = True


        self.output('lspeed', self.lspeed)
        self.output('rspeed', self.rspeed)
        self.drive.tankDrive(self.lspeed, self.rspeed)


  
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
