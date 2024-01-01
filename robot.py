#!/usr/bin/env python3
import typing
import wpilib
from wpimath.geometry import Rotation2d 
import commands2
import ctre
import math
import constants
from constants import ModuleConstants

from robotcontainer import RobotContainer
from commands2 import CommandScheduler
from commands.TeleopCommands.SwerveJoystickCmd import SwerveJoystickCmd
import ntcore
import robotpy_apriltag
from wpilib import Timer

# -----------------
# code adopted from FRC 0 to Autonomous: #6 Swerve Drive Auto (https://www.youtube.com/watch?v=0Xi9yb1IMyA)
# and ported to Python
# if we can't get it to work,
# we can use the code from https://github.com/1757WestwoodRobotics/RobotBase/tree/master
# -----------------

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
        self.swerve = self.Container.swerve
        CommandScheduler.getInstance().registerSubsystem(self.swerve)
        #~ LED commands and variables
        self.LEDserver = wpilib.I2C(wpilib.I2C.Port.kMXP, 100)
        self.previousLEDCommand = 0
    

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        # self.testabsoluteEncoder = wpilib.DutyCycleEncoder(5)
        self.swerve.frontRight.resetEncoders()
        self.swerve.frontLeft.resetEncoders()
        self.swerve.backLeft.resetEncoders()
        self.swerve.backRight.resetEncoders()



    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        # self.sd.putNumber("absEncoder", self.testabsoluteEncoder.getAbsolutePosition())
        # print(self.testabsoluteEncoder.getAbsolutePosition())
                
        # print(self.swerve.frontRight.getTurningPostion())
        # self.sd.putNumber(f"turning position FL(rad, AbsEnc)", self.swerve.frontLeft.getAbsoluteEncoderRad())
        # self.sd.putNumber(f"turning position FR(rad, AbsEnc)", self.swerve.frontRight.getAbsoluteEncoderRad())
        # self.sd.putNumber(f"turning position BL(rad, AbsEnc)", self.swerve.backLeft.getAbsoluteEncoderRad())
        # self.sd.putNumber(f"turning position BR(rad, AbsEnc)", self.swerve.backRight.getAbsoluteEncoderRad())

        # self.sd.putNumber(f"turning position FL(rad, MotEnc)", self.swerve.frontLeft.getTurningPostion())
        # self.sd.putNumber(f"turning position FR(rad, MotEnc)", self.swerve.frontRight.getTurningPostion())
        # self.sd.putNumber(f"turning position BL(rad, MotEnc)", self.swerve.backLeft.getTurningPostion())
        # self.sd.putNumber(f"turning position BR(rad, MotEnc)", self.swerve.backRight.getTurningPostion())

        # self.swerve.sd.putNumber("ActualFL", float(self.swerve.getModuleStates()[0].angle.degrees()))
        # self.swerve.sd.putNumber("ActualFR", float(self.swerve.getModuleStates()[1].angle.degrees()))
        # self.swerve.sd.putNumber("ActualBL", float(self.swerve.getModuleStates()[2].angle.degrees()))
        # self.swerve.sd.putNumber("ActualBR", float(self.swerve.getModuleStates()[3].angle.degrees()))

            


    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        #~ will be needed for future use
        self.autonomousCommand = self.Container.getAutonomousCommand()

        # self.output("ato com", self.autonomousCommand)
       
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        try:
            #make a function that constantly updates robot pose/gyro based on apriltags
            #updatelocaiton()
            pass
        except:
            if not self.ds.isFMSAttached():
                raise


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

        self.swerve.frontLeft.drivingEncoder.setPosition(0)
        self.swerve.frontRight.drivingEncoder.setPosition(0)
        self.swerve.backLeft.drivingEncoder.setPosition(0)
        self.swerve.backRight.drivingEncoder.setPosition(0)

    def teleopPeriodic(self):
        self.swerve.frontRight.resetEncoders()
        self.swerve.frontLeft.resetEncoders()
        self.swerve.backLeft.resetEncoders()
        self.swerve.backRight.resetEncoders()
        #TODO put code in try and except functions, shown here https://robotpy.readthedocs.io/en/stable/guide/guidelines.html#don-t-die-during-the-competition
        try:
            # self.sd.putNumber(f"turning position FL(rad, AbsEnc)", self.swerve.frontLeft.getAbsoluteEncoderRad())
            # self.sd.putNumber(f"turning position FR(rad, AbsEnc)", self.swerve.frontRight.getAbsoluteEncoderRad())
            # self.sd.putNumber(f"turning position BL(rad, AbsEnc)", self.swerve.backLeft.getAbsoluteEncoderRad())
            # self.sd.putNumber(f"turning position BR(rad, AbsEnc)", self.swerve.backRight.getAbsoluteEncoderRad())

            # self.sd.putNumber(f"turning position FL(rad, MotEnc)", self.swerve.frontLeft.getTurningPostion())
            # self.sd.putNumber(f"turning position FR(rad, MotEnc)", self.swerve.frontRight.getTurningPostion())
            # self.sd.putNumber(f"turning position BL(rad, MotEnc)", self.swerve.backLeft.getTurningPostion())
            # self.sd.putNumber(f"turning position BR(rad, MotEnc)", self.swerve.backRight.getTurningPostion())
            # print(Timer.getFPGATimestamp(),  
            #       round(self.swerve.frontRight.getDrivingPosition(), 4), round(self.swerve.frontRight.getDrivingVelocity(), 4), 
            #       round(self.swerve.backLeft.getDrivingPosition(), 4), round(self.swerve.backLeft.getDrivingVelocity(), 4),
            #       round(self.swerve.backRight.getDrivingPosition(), 4), round(self.swerve.backRight.getDrivingVelocity(), 4), sep=",")
            self.sd.putNumber("Module Position (FL)", self.swerve.frontLeft.getDrivingPosition())
            self.sd.putNumber("Module Position (FR)", self.swerve.frontRight.getDrivingPosition())
            self.sd.putNumber("Module Position (BL)", self.swerve.backLeft.getDrivingPosition())
            self.sd.putNumber("Module Position (BR)", self.swerve.backRight.getDrivingPosition())

            self.sd.putNumber("Module Velocity (FL)", self.swerve.frontLeft.getDrivingVelocity())
            self.sd.putNumber("Module Velocity (FR)", self.swerve.frontRight.getDrivingVelocity())
            self.sd.putNumber("Module Velocity (BL)", self.swerve.backLeft.getDrivingVelocity())
            self.sd.putNumber("Module Velocity (BR)", self.swerve.backRight.getDrivingVelocity())

            
        except:
            if not self.ds.isFMSAttached():
                raise
        

        # print(wpilib.DriverStation.getAlliance())

        # self.sd.putNumber("gyroYaw", self.swerve.getHeading())
        # self.sd.putValue("gyroPitch", self.swerve.getPose())

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
        pass
    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
        self.swerve.frontRight.resetEncoders()
        self.swerve.frontLeft.resetEncoders()
        self.swerve.backLeft.resetEncoders()
        self.swerve.backRight.resetEncoders()
    def testPeriodic(self) -> None:
        pass
            
    
    def sendLEDCommand(self, command, isRedAlliance = None):
            # send the specified command to the LEDserver
            team_command = 0
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
