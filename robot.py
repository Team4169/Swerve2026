#!/usr/bin/env python3
#TODO:
#? 1.FIX climb
#? 2 test out knoking algae

#? 2. Continue with voltagelimiter! (Can do it in the Rev Client here:
#? 3. Test auto/pathplanner + on-the-fly
#! IMPORTANT: alternate method for limiters using the rev hardware client https://github.com/CrimsonRobotics/Current-Limiting-Spark-MAX?tab=readme-ov-file#readme
#? 4. Driver Practice!!

#* "that sets that up" - Luc Sciametta 4:16pm 3/4/2024 (mikhail wrote this)
#* " we still have this quote" - Annie Huang 1/22/2025 (grady wrote this)
#* "Okay whats the quotes?" - Ofir van Creveld 1/31/2025 (adam wrote this)
#* "This thing goes down and stops the motor" - Grady May 1/31/2025 (annie wrote this)
#* "We need 6 of the same file. trust." - Adam Mokdad 1/31/2025 (ofir wrote this)
#* "lets call it floppy something." - Ofir van Creveld 1/31/2025 (adam wrote this)
#* "Grady can never be wrong" -Annie Huang 2/4/2025 (grady wrote this)
#* "if only i had a log" -ofir 3/3/2025 (grady wrote this)
#* "maybe we put all the code on one line" - Adam Mokdad 3/10/2025 (ofir wrote this)
#* "GET BACK TO WORK" - Grady May 3/10/2025 (ofir wrote this)

import typing, wpilib, ntcore
from wpimath.geometry import Pose2d
import commands2
from robotcontainer import RobotContainer
from commands2 import CommandScheduler
from wpilib import Timer
from wpilib import Field2d
from wpilib.cameraserver import CameraServer

import phoenix6

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

        # self.drive = self.Container.drive
        self.swerve = self.Container.swerve
        CommandScheduler.getInstance().registerSubsystem(self.swerve)

        # Init the camera server
        CameraServer().launch()

        #~ LED commands and variables
        # self.LEDserver = wpilib.I2C(wpilib.I2C.Port.kMXP, 100)
        # self.previousLEDCommand = 0

        self.network_tables = ntcore.NetworkTableInstance.getDefault()
        self.camera_tables = self.network_tables.getTable("SmartDashboard")

        #self.slew_limiter = SlewRateLimiter(1.0)  # 1 volt per second

        # Assume this is a motor controller (replace with your actual motor controller)
        #self.motor_controller = MotorController(0)


        self.field = Field2d()
        self.sd.putData("Field", self.field)


    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        # self.testabsoluteEncoder = wpilib.DutyCycleEncoder(5)
        self.swerve.frontRight.resetEncoders()   
        self.swerve.frontLeft.resetEncoders()
        self.swerve.backLeft.resetEncoders()
        self.swerve.backRight.resetEncoders()


    def disabledPeriodic(self) -> None:
        
        self.jetson1X = self.camera_tables.getEntry("x1").getDouble(0)
        self.jetson2X = self.camera_tables.getEntry("x2").getDouble(0)
        self.jetson1Y = self.camera_tables.getEntry("y1").getDouble(0)
        self.jetson2Y = self.camera_tables.getEntry("y2").getDouble(0)
        self.jetson1weight = self.camera_tables.getEntry("w1").getDouble(0)
        self.jetson2weight = self.camera_tables.getEntry("w2").getDouble(0)
        self.jetson1rot = self.camera_tables.getEntry("r1").getDouble(0)
        self.jetson2rot = self.camera_tables.getEntry("r2").getDouble(0)
        self.timeSinceLastDataReceivedFromJetson1 = self.camera_tables.getEntry("w1").getLastChange() - ntcore._now()
        self.timeSinceLastDataReceivedFromJetson2 = self.camera_tables.getEntry("w2").getLastChange() - ntcore._now()

        self.robot_pose = Pose2d(self.jetson2X, self.jetson2Y, self.jetson2rot)
        self.field.setRobotPose(self.robot_pose)

        # self.sd.putNumber("w2", self.jetson2weight)

        # print("w2 _______________", self.jetson2weight)

        # if self.timeSinceLastDataReceivedFromJetson1 > 2: self.jetson1weight = 0
        # if self.timeSinceLastDataReceivedFromJetson2 > 2: self.jetson2weight = 0
        # if self.jetson1weight + self.jetson2weight > 0:
            
        #     self.xAve = (self.jetson1X * self.jetson1weight + self.jetson2X * self.jetson2weight) / (self.jetson1weight + self.jetson2weight)
        #     self.yAve = (self.jetson1Y * self.jetson1weight + self.jetson2Y * self.jetson2weight) / (self.jetson1weight + self.jetson2weight)
        
        #     self.rotAve = math.atan2(math.sin(self.jetson1rot) * self.jetson1weight + math.sin(self.jetson2rot) * self.jetson2weight, math.cos(self.jetson1rot) * self.jetson1weight + math.cos(self.jetson2rot) * self.jetson2weight)
        #     # print('\n*$*$*$*$*\n',f"OurX: {self.xAve}, OurY: {self.yAve}, ConstX: {RobotConstants.speakerXPosition}, ConstY: {RobotConstants.speakerYPosition}")
        #     if True: #on red team
        #         self.xDistance = -RobotConstants.speakerXPosition - self.xAve #8.3m
        #     else: #on blue team
        #         self.xDistance = RobotConstants.speakerXPosition - self.xAve #8.3m
        #     self.yDistance = RobotConstants.speakerYPosition - self.yAve #1.45m
        #     self.distanceToOurSpeaker = math.sqrt(self.xDistance**2 + self.yDistance**2)
            # print(self.distanceToShooter)

           # print(self.distanceToShooter)

        """This function is called periodically when disabled"""
        # self.sd.putNumber("absEncoder", self.testabsoluteEncoder.getAbsolutePosition())
        # print(self.testabsoluteEncoder.getAbsolutePosition())
                
        # print(self.swerve.frontRight.getTurningPostion())

        self.sd.putNumber(f"turning position FL(rad, AbsEnc)", self.swerve.frontLeft.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position FR(rad, AbsEnc)", self.swerve.frontRight.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position BL(rad, AbsEnc)", self.swerve.backLeft.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position BR(rad, AbsEnc)", self.swerve.backRight.getAbsoluteEncoderRad())

        self.sd.putNumber(f"turning position FL(rad, MotEnc)", self.swerve.frontLeft.getTurningPostion())
        self.sd.putNumber(f"turning position FR(rad, MotEnc)", self.swerve.frontRight.getTurningPostion())
        self.sd.putNumber(f"turning position BL(rad, MotEnc)", self.swerve.backLeft.getTurningPostion())
        self.sd.putNumber(f"turning position BR(rad, MotEnc)", self.swerve.backRight.getTurningPostion())

        self.swerve.sd.putNumber("ActualFL", float(self.swerve.getModuleStates()[0].angle.degrees()))
        self.swerve.sd.putNumber("ActualFR", float(self.swerve.getModuleStates()[1].angle.degrees()))
        self.swerve.sd.putNumber("ActualBL", float(self.swerve.getModuleStates()[2].angle.degrees()))
        self.swerve.sd.putNumber("ActualBR", float(self.swerve.getModuleStates()[3].angle.degrees()))
        # self.sd.putBoolean("inMidstage", self.camera_tables.getEntry("inMid").getValue())



    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

        self.swerve.frontLeft.drivingEncoder.setPosition(0)
        self.swerve.frontRight.drivingEncoder.setPosition(0)
        self.swerve.backLeft.drivingEncoder.setPosition(0)
        self.swerve.backRight.drivingEncoder.setPosition(0)
        
        self.autonomousCommand = self.Container.getAutonomousCommand()


        #self.output("ato com", self.autonomousCommand)
        # print("***********************************************************")
        if self.autonomousCommand:
            # print(self.autonomousCommand, "--------------------------------------------")
            self.autonomousCommand.schedule()
        '''
        changeHorizontal = 1000
        changeDistance = 1000

        changeRot = math.atan2(changeHorizontal, changeDistance)
        targetPose = Pose2d(changeDistance, changeHorizontal, Rotation2d.fromRotations(changeRot / (math.pi * 2)))

        pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            AutoConstants.constraints,
            goal_end_vel=0.0,
            rotation_delay_distance=0.0
        )

        if pathfindingCommand:
            pathfindingCommand.schedule()

        # self.autoSelected = self.chooser.getSelected()
        # print("Auto selected: " + self.autoSelected)
        '''
            
    def autonomousPeriodic(self) -> None:
        # print("speed: " + str(self.state.speed / RobotConstants.kphysicalMaxSpeedMetersPerSecond))
        self.swerve.frontRight.resetEncoders()
        self.swerve.frontLeft.resetEncoders()
        self.swerve.backLeft.resetEncoders()
        self.swerve.backRight.resetEncoders()

        self.sd.putNumber("Back Left Encoder Position: ", self.swerve.backLeft.absoluteEncoder.get_position().value_as_double)
        self.sd.putNumber("Back Right Encoder Position: ", self.swerve.backRight.absoluteEncoder.get_position().value_as_double)
        self.sd.putNumber("Front Left Encoder Position: ", self.swerve.frontLeft.absoluteEncoder.get_position().value_as_double)
        self.sd.putNumber("Front Right Encoder Position: ", self.swerve.frontRight.absoluteEncoder.get_position().value_as_double)

        self.sd.putNumber(f"turning position FL(rad, AbsEnc)", self.swerve.frontLeft.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position FR(rad, AbsEnc)", self.swerve.frontRight.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position BL(rad, AbsEnc)", self.swerve.backLeft.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position BR(rad, AbsEnc)", self.swerve.backRight.getAbsoluteEncoderRad())

        self.swerve.sd.putNumber("ActualFL", float(self.swerve.getModuleStates()[0].angle.degrees()))
        self.swerve.sd.putNumber("ActualFR", float(self.swerve.getModuleStates()[1].angle.degrees()))
        self.swerve.sd.putNumber("ActualBL", float(self.swerve.getModuleStates()[2].angle.degrees()))
        self.swerve.sd.putNumber("ActualBR", float(self.swerve.getModuleStates()[3].angle.degrees()))




        """This function is called periodically during autonomous"""

        try:
            # #make a function that constantly updates robot pose/gyro based on apriltags
            # updatelocaiton()
            pass
        except:
            if not self.ds.isFMSAttached():
                raise


    def teleopInit(self) -> None:

        self.ds = wpilib.DriverStation
 

        # wpilib.CameraServer.launch()
        self.isRedAlliance = self.ds.getAlliance() == self.ds.Alliance.kRed
        #self.sendLEDCommand(3, self.isRedAlliance)
        # self.drive.resetEncoders()
        # self.moveRestriction = 1
        
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        # if self.AutonomousCommand:
        #     self.AutonomousCommand.cancel()
        
        self.swerve.frontLeft.drivingEncoder.setPosition(0)
        self.swerve.frontRight.drivingEncoder.setPosition(0)
        self.swerve.backRight.drivingEncoder.setPosition(0)
        self.swerve.backLeft.drivingEncoder.setPosition(0)

        self.sd.putNumber("BackLeft Starting Turning Position", self.swerve.backLeft.getTurningPostion())
        self.sd.putNumber("BackRight Starting Turning Position", self.swerve.backRight.getTurningPostion())
        self.sd.putNumber("FrontLeft Starting Turning Position", self.swerve.frontLeft.getTurningPostion())
        self.sd.putNumber("FrontRight Starting Turning Position", self.swerve.frontRight.getTurningPostion())

        # print("Starting teleop...")
        # self.speed = 0
        self.coral = self.Container.coral
        self.algae = self.Container.algae
        self.lastTimeStamp = None

        self.algae.algaeLiftMotor.getEncoder().setPosition(0)

    def teleopPeriodic(self):
        #print("algae height:", self.algae.algaeLiftMotor.getEncoder().getPosition()) #max should be 19.2 (its actually 14.33)

        #makes stopLiftCoral execute after a certain amount of time. used to make coral lift to a specific height
        if self.coral.out == True:
            self.algae.canRun = False
        if self.coral.out == False:
            self.algae.canRun = True
        if self.algae.out == True:
            self.coral.canRun = False
        if self.algae.out == False:
            self.coral.canRun = True

        if self.coral.liftingCoral:
            if self.lastTimeStamp:
                deltaTime = Timer.getTimestamp() - self.lastTimeStamp
            else:
                deltaTime = 0
            self.coral.liftCoralTimer -= deltaTime
            self.lastTimeStamp = Timer.getTimestamp()
            # print(self.coral.liftCoralTimer)
            
            if self.coral.liftCoralTimer <=0:
                self.coral.stopLiftCoral()

        self.swerve.frontRight.resetEncoders()
        self.swerve.frontLeft.resetEncoders()
        self.swerve.backLeft.resetEncoders()
        self.swerve.backRight.resetEncoders()

        #voltageLimiter = SlewRateLimiter(0.5)

        #self.sd.putNumber(voltageLimiter.calculate(self.SwerveJoystickCmd.xSpeed))
        # self.sd.putNumber(voltageLimiter.calculate(self.SwerveJoystickCmd.ySpeed))
        # self.sd.putNumber(voltageLimiter.calculate(self.SwerveJoystickCmd.zRotation))
        
        #Current Limiter 
        # self.sd.putNumber("Current Motor Output", drivingmotor.getOutputCurrent())
        

        self.sd.putNumber("BackLeft Turning Position", self.swerve.backLeft.getTurningPostion())
        self.sd.putNumber("BackRight Turning Position", self.swerve.backRight.getTurningPostion())
        self.sd.putNumber("FrontLeft Turning Position", self.swerve.frontLeft.getTurningPostion())
        self.sd.putNumber("FrontRight Turning Position", self.swerve.frontRight.getTurningPostion())

        self.swerve.sd.putNumber("ActualFL", float(self.swerve.getModuleStates()[0].angle.degrees()))
        self.swerve.sd.putNumber("ActualFR", float(self.swerve.getModuleStates()[1].angle.degrees()))
        self.swerve.sd.putNumber("ActualBL", float(self.swerve.getModuleStates()[2].angle.degrees()))
        self.swerve.sd.putNumber("ActualBR", float(self.swerve.getModuleStates()[3].angle.degrees()))

        # self.jetson1X = self.camera_tables.getEntry("x1").getDouble(0)
        # self.jetson2X = self.camera_tables.getEntry("x2").getDouble(0)
        # self.jetson1Y = self.camera_tables.getEntry("y1").getDouble(0)
        # self.jetson2Y = self.camera_tables.getEntry("y2").getDouble(0)
        # self.jetson1weight = self.camera_tables.getEntry("w1").getDouble(0)
        # self.jetson2weight = self.camera_tables.getEntry("w2").getDouble(0)
        # self.jetson1rot = self.camera_tables.getEntry("r1").getDouble(0)
        # self.jetson2rot = self.camera_tables.getEntry("r2").getDouble(0)
        # self.timeSinceLastDataReceivedFromJetson1 = self.camera_tables.getEntry("w1").getLastChange() - ntcore._now()
        # self.timeSinceLastDataReceivedFromJetson2 = self.camera_tables.getEntry("w2").getLastChange() - ntcore._now()
        # if self.timeSinceLastDataReceivedFromJetson1 > 2: self.jetson1weight = 0
        # if self.timeSinceLastDataReceivedFromJetson2 > 2: self.jetson2weight = 0
        # if self.jetson1weight + self.jetson2weight > 0:
            
        #     self.xAve = (self.jetson1X * self.jetson1weight + self.jetson2X * self.jetson2weight) / (self.jetson1weight + self.jetson2weight)
        #     self.yAve = (self.jetson1Y * self.jetson1weight + self.jetson2Y * self.jetson2weight) / (self.jetson1weight + self.jetson2weight)
        
        #     self.rotAve = math.atan2(math.sin(self.jetson1rot) * self.jetson1weight + math.sin(self.jetson2rot) * self.jetson2weight, math.cos(self.jetson1rot) * self.jetson1weight + math.cos(self.jetson2rot) * self.jetson2weight)
        #     # print('\n*$*$*$*$*\n',f"OurX: {self.xAve}, OurY: {self.yAve}, ConstX: {RobotConstants.speakerXPosition}, ConstY: {RobotConstants.speakerYPosition}")
        #     if self.isRedAlliance: #on red team
        #         self.xDistance = -RobotConstants.speakerXPosition - self.xAve #8.3m
        #     else: #on blue team
        #         self.xDistance = RobotConstants.speakerXPosition - self.xAve #8.3m
        #     self.yDistance = RobotConstants.speakerYPosition - self.yAve #1.45m
        #     self.distanceToOurSpeaker = math.sqrt(self.xDistance**2 + self.yDistance**2)
        #     # print(self.distanceToShooter)
        # # print(self.Container.autoShooterWarmup)

        # # if self.distanceToShooter <= 3: # and self.Container.autoShooterWarmup:
        # #     self.Container.shooter.runShooter()

        # # self.sd.putNumber("drivingLimiter", DrivingConstants.drivingSpeedLimiter)

        # # if self.operatorController.getLeftTriggerAxis() > 0.2:
        # #     self.Container.climber.runRightClimbingMotor(0.35)
        # # else:
        # #     self.Container.climber.stopRightClimbingMotor()
        
        # # if self.operatorController.getRightTriggerAxis() > 0.2:
        # #     self.Container.climber.runLeftClimbingMotor(-0.35)

        # # else:
        # #     self.Container.climber.stopLeftClimbingMotor()

        # # if self.operatorController.getLeftBumperPressed():
        # #     self.Container.climber.runRightClimbingMotor(-0.35)
        # # else:
        # #     self.Container.climber.stopRightClimbingMotor()
            
        # if self.operatorController.getRightBumperPressed():
        #     self.Container.climber.runLeftClimbingMotor(-0.35)
        # else:
        #     self.Container.climber.stopLeftClimbingMotor()

        
        
        # Attempt to code dpad (change it to another button if it doesn't work) 
        # Turn robot until arctan(xDistance/yDistance) = 90 degrees
            
        # if self.operatorController.POVRightPressed():
        #     self.rotation = Rotation2d() 
        #     self.swerve.rotateToSpeaker(self.rotation)

        #TODO put code in try and except functions, shown here https://robotpy.readthedocs.io/en/stable/guide/guidelines.html#don-t-die-during-the-competition
        # try:
        self.sd.putNumber(f"turning position FL(rad, AbsEnc)", self.swerve.frontLeft.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position FR(rad, AbsEnc)", self.swerve.frontRight.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position BL(rad, AbsEnc)", self.swerve.backLeft.getAbsoluteEncoderRad())
        self.sd.putNumber(f"turning position BR(rad, AbsEnc)", self.swerve.backRight.getAbsoluteEncoderRad())

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

        self.sd.putNumber("Back Left Abs Encoder: ", self.swerve.backLeft.absoluteEncoder.get_absolute_position().value_as_double)
        self.sd.putNumber("Back Right Abs Encoder: ", self.swerve.backRight.absoluteEncoder.get_absolute_position().value_as_double)
        self.sd.putNumber("Front Left Abs Encoder: ", self.swerve.frontLeft.absoluteEncoder.get_absolute_position().value_as_double)
        self.sd.putNumber("Front Right Abs Encoder: ", self.swerve.frontRight.absoluteEncoder.get_absolute_position().value_as_double)

        self.sd.putNumber("Back Left Encoder Position: ", self.swerve.backLeft.absoluteEncoder.get_position().value_as_double)
        self.sd.putNumber("Back Right Encoder Position: ", self.swerve.backRight.absoluteEncoder.get_position().value_as_double)
        self.sd.putNumber("Front Left Encoder Position: ", self.swerve.frontLeft.absoluteEncoder.get_position().value_as_double)
        self.sd.putNumber("Front Right Encoder Position: ", self.swerve.frontRight.absoluteEncoder.get_position().value_as_double)

        self.sd.putNumber("Back Left Encoder Velocity: ", self.swerve.backLeft.absoluteEncoder.get_velocity().value_as_double)
        self.sd.putNumber("Back Right Encoder Velocity: ", self.swerve.backRight.absoluteEncoder.get_velocity().value_as_double)
        self.sd.putNumber("Front Left Encoder Velocity: ", self.swerve.frontLeft.absoluteEncoder.get_velocity().value_as_double)
        self.sd.putNumber("Front Right Encoder Velocity: ", self.swerve.frontRight.absoluteEncoder.get_velocity().value_as_double)

        self.sd.putNumber("Back Left Encoder Supply Voltage: ", self.swerve.backLeft.absoluteEncoder.get_supply_voltage().value_as_double)
        self.sd.putNumber("Back Right Encoder Supply Voltage: ", self.swerve.backRight.absoluteEncoder.get_supply_voltage().value_as_double)
        self.sd.putNumber("Front Left Encoder Supply Voltage: ", self.swerve.frontLeft.absoluteEncoder.get_supply_voltage().value_as_double)
        self.sd.putNumber("Front Right Encoder Supply Voltage: ", self.swerve.frontRight.absoluteEncoder.get_supply_voltage().value_as_double)

        # except:
            # if not self.ds.isFMSAttached():
            #     raise
        
    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
        self.swerve.frontRight.resetEncoders()
        self.swerve.frontLeft.resetEncoders()
        self.swerve.backLeft.resetEncoders()
        self.swerve.backRight.resetEncoders()

    def testPeriodic(self) -> None:
        pass
                

if __name__ == "__main__":
    wpilib.run(MyRobot)


    
# #!/usr/bin/env python3

# robotpy-commands-v2==2024.3.1
# robotpy-pathplannerlib==2024.2.3

 #robotpy_version = "2024.3.1.0"