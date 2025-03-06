import wpilib, wpimath
from wpilib.interfaces import GenericHID
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

import ntcore, rev, commands2
import commands2.cmd
import commands2.button

import constants
from constants import AutoConstants, OIConstants, RobotConstants, DrivingConstants

from commands.TeleopCommands.SwerveJoystickCmd import SwerveJoystickCmd
from commands2.button.trigger import Trigger
from commands2.button import JoystickButton, CommandXboxController
from wpilib import XboxController, Joystick

from subsystems.swervesubsystem import SwerveSubsystem
from subsystems.algaeSubsystem import AlgaeSubsystem
from subsystems.climbingSubsystem import ClimbingSubsystem
from subsystems.coralSubsystem import CoralSubsystem

#from subsystems.armsubsystem import ArmSubsystem
# from subsystems.intakeSubsystem import IntakeSubsystem
# from subsystems.midstageSubsystem import MidstageSubsystem
#from subsystems.ShooterSubsystem import ShooterSubsystem

import math
# import photonvision

from pathplannerlib.auto import NamedCommands, PathPlannerAuto, AutoBuilder, PathPlannerPath, PathConstraints
from pathplannerlib.path import GoalEndState

# from commands.testcommands.move1module import move1module
# from commands.testcommands.move2motors import move2motors
# from commands.testcommands.move4modules import move4modules
# from commands.testcommands.MoveInACircle import MoveInACircle
# from commands.TeleopCommands.driveWaypoint import DriveWaypoint

from commands.testcommands.rotateToSpeakerCommand import rotateToSpeakerCommand

# from commands.AutonCommands.sCurve import sCurve
from commands2 import InstantCommand

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort) # can also use ps4 controller (^v^)
        self.operatorController = wpilib.XboxController(OIConstants.kArmControllerPort)

        # The robot's subsystems
        self.swerve = SwerveSubsystem() #! test #? test what
        self.algae = AlgaeSubsystem()
        self.climber = ClimbingSubsystem()
        self.coral = CoralSubsystem()

        # self.intake = IntakeSubsystem()
        # self.midstage = MidstageSubsystem()
        # self.shooter = ShooterSubsystem()

      

        self.swerve.setDefaultCommand(SwerveJoystickCmd(
            swerve=self.swerve,
            driverController=self.driverController
        ))
        
        #^^Added this today (1/11)
        #commands for pathplanner
        NamedCommands.registerCommand("resetOdometry",
            commands2.InstantCommand(lambda:self.swerve.resetOdometry(self.trajectory.initialPose()))
        )
        NamedCommands.registerCommand("stopModules",
            commands2.InstantCommand(lambda:self.swerve.stopModules())
        )
        NamedCommands.registerCommand("liftCoral",
            commands2.InstantCommand(lambda:self.coral.liftCoral(0.75))
        )
        NamedCommands.registerCommand("lowerCoral", #? Ofir claims it goes down by itself so this isnt needed. check and make sure
            commands2.InstantCommand(lambda:self.coral.liftCoral(-0.75))
        )
        NamedCommands.registerCommand("stopLiftCoral",
            commands2.InstantCommand(lambda:self.coral.stopLiftCoral())
        )
        NamedCommands.registerCommand("openCoral",
             commands2.InstantCommand(lambda:self.coral.runDepositCoral(0.75)) # check if positive is open and negative is close
        )
        NamedCommands.registerCommand("stopCoral",
             commands2.InstantCommand(lambda:self.coral.stopDepositCoral())
        )
        NamedCommands.registerCommand("liftAlgae",
            commands2.InstantCommand(lambda:self.algae.runLiftAlgae(0.75))                          
        )
        NamedCommands.registerCommand("stopLiftAlgae",
            commands2.InstantCommand(lambda:self.algae.stopLiftAlgae())                          
        )
        NamedCommands.registerCommand("runAlgae",
            commands2.InstantCommand(lambda:self.algae.runAlgae(0.75))                          
        )
        NamedCommands.registerCommand("stopAlgae",
            commands2.InstantCommand(lambda:self.algae.stopAlgae())                          
        )
        # NamedCommands.registerCommand("stopMidstage",
        #     commands2.InstantCommand(lambda:self.midstage.stopMidstage())
        # )
        # NamedCommands.registerCommand("setShooterAngle",
        #     commands2.InstantCommand(lambda:self.shooter.setShooterAngle(self.shooter.getShooterAngle()))
        # )
        # NamedCommands.registerCommand("stopShooter",
        #     commands2.InstantCommand(lambda:self.shooter.stopShooter())
        # )
        # NamedCommands.registerCommand("shootRing",
        #     commands2.InstantCommand(lambda:self.shooter.runShooter())
        # )

       
        #self.driveWaypointCommand = DriveWaypoint(self.swerve)
        self.configureButtonBindings()

        # sendable chooser
        #self.chooser = wpilib.SendableChooser()

        self.autoChooser = AutoBuilder.buildAutoChooser()

        self.putCoralInReef = "Put Coral in Reef"
        self.knockOutAndScore = "Knock out and score"
        self.escape = "Escape"
        self.knockOutCoral = "Knock out Coral"
        self.escapeBottom = "Escape Bottom"
        self.pickupAlgaeandScoreB = "Pickup Algae + Score Bottom"
        self.pickupAlgaeandScore = "Pickup Algae + Score"
        self.moveForward = "Move two meters forward"
        self.spinForward = "Spin two meters forward"
        
        # self.NewAuto = "New Auto"


        self.autoChooser.addOption("Put Coral in Reef", self.putCoralInReef)
        self.autoChooser.addOption("Knock out and score", self.knockOutAndScore)
        self.autoChooser.addOption("Escape", self.escape)
        self.autoChooser.addOption("Knock out Coral", self.knockOutCoral)
        self.autoChooser.addOption("Escape Bottom", self.escapeBottom)
        self.autoChooser.addOption("Pickup Algae + Score Bottom", self.pickupAlgaeandScoreB)
        self.autoChooser.addOption("Pickup Algae + Score", self.pickupAlgaeandScore)
        self.autoChooser.addOption("Move two meters forward", self.moveForward)
        self.autoChooser.addOption("Spin two meters forward", self.spinForward)
        #self.autoChooser.addOption("New Auto", self.NewAuto)
       
       
        # # # Put the autoChooser on the dashboard
        self.sd = wpilib.SmartDashboard
        self.sd.putData("Autonomousff", self.autoChooser)

        # self.network_tables = ntcore.NetworkTableInstance.getDefault()
        #self.datatable = self.network_tables.getTable("datatable")

        # self.autoShooterWarmup = True
        # self.sd.putData("AutoShooterWarmup", self.autoShooterWarmup)
        
        self.climberSpeed = RobotConstants.climberSpeed
        self.algaeIntakeSpeed = RobotConstants.algaeIntakeSpeed
        self.algaeLiftSpeed = RobotConstants.algaeLiftSpeed
        self.coralLiftSpeed = RobotConstants.coralLiftSpeed
        self.coralDepositSpeed = RobotConstants.coralDepositSpeed
        self.coralL1Time = RobotConstants.coralL1Time
        self.coralL2Time = RobotConstants.coralL2Time
        self.coralL3Time = RobotConstants.coralL3Time
    
    # def runObjectDetectionPath(self):
    #     start_pose = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    #     end_pose = Pose2d(10, 0, Rotation2d.fromDegrees(0))

    #     bezier_points = PathPlannerPath.bezierFromPoses([start_pose, end_pose])
    #     path = PathPlannerPath(
    #         bezier_points,
    #         AutoConstants.constraints,
    #         GoalEndState(0, Rotation2d.fromDegrees(0))
    #     )
    #     return AutoBuilder.followPath(path)

    def setSlowMode(self): #-> commands2.Command
        self.sd.putString("Slow Mode", "ON")
        DrivingConstants.drivingSpeedLimiter = 0.5
        DrivingConstants.rotationSpeedLimiter = 0.5
        # return self.swerve.driveChassisSpeeds(self.drivingLimiter)

    def unbindSlowMode(self):
        self.sd.putString("Slow Mode", "OFF")
        DrivingConstants.drivingSpeedLimiter = 1
        DrivingConstants.rotationSpeedLimiter = 1

    def getAutonomousCommand(self):
        """Returns the autonomous command to run"""
        # path = self.autoChooser.getSelected()
        path = 'Escape'
        print(f"selected path is: {path}")
        return PathPlannerAuto(path)

    def getTeleopCommand(self):
        self.sd.putString("Starting ontheFlyPath", "Start")
        print("Starting")
        auto = self.onTheFlyPathTest()
        auto.schedule()

    def stopTeleopCommand(self):
        self.sd.putString("Terminating onTheFlyPath", "Stop")
        print("Stop")
        PathPlannerAuto.cancel(self.onTheFlyPathTest()) #This does indeed cancel the on the fly path, but it just starts over again in an infinite loop
        print("Cancelled")
    
        #set speed to zero? Test it next time!

    
    def onTheFlyPathTest(self):
        print("starting on the fly path now")

        waypoints = PathPlannerPath.waypointsFromPoses([
            Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0))
        ])
        constraints = PathConstraints(0.5, 1, 0.5 * math.pi, 1 * math.pi) # The constraints for this path.
        # constraints = PathConstraints.unlimitedConstraints(12.0) # You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        # Create the path using the waypoints created above
        path = PathPlannerPath(
            waypoints,
            constraints, #AutoConstants.constaints
            None, # The ideal starting state, this is only relevant for pre-planned paths, so can be None for on-the-fly paths.
            GoalEndState(0.0, Rotation2d.fromDegrees(0)) # Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        )

        # Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = True
        auto = AutoBuilder.pathfindThenFollowPath(path, constraints)
        return auto

        
    def configureButtonBindings(self):
        
        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kStart).onTrue(InstantCommand(lambda: self.swerve.zeroHeading()))
        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kBack).whileTrue(InstantCommand(lambda: self.swerve.lockWheels())).onFalse(lambda: self.swerve.unlockWheels())
        
        # # Engage Object Detection
        # commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kA).onTrue(self.driveWaypointCommand)
        # commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kA).onTrue(InstantCommand(lambda: self.runObjectDetectionPath()))
        
        # Toggle Slow Mode
        # DRIVER
        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kRightBumper).whileTrue(InstantCommand(lambda: self.setSlowMode())).onFalse(InstantCommand(lambda: self.unbindSlowMode()))

        #AUTO SETUPS (using on-the-fly)
        #Uncomment one or the other, do not uncomment both!!
        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kA).whileTrue(InstantCommand(lambda: self.getTeleopCommand())).whileFalse(InstantCommand(lambda: self.stopTeleopCommand())) # hopefully terminates on the fly path
        #commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kA).onTrue(InstantCommand(lambda: self.getTeleopCommand())) #Just accelerates robot indefinitly
    
        # OPERATOR

        #CORAL
        #lifting coral 
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).onTrue(InstantCommand(lambda: self.coral.liftCoral(self.coralLiftSpeed, self.coralL1Time)))
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).onTrue(InstantCommand(lambda: self.coral.liftCoral(self.coralLiftSpeed, self.coralL2Time)))
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).onTrue(InstantCommand(lambda: self.coral.liftCoral(self.coralLiftSpeed, self.coralL3Time)))
        #descending coral
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).whileTrue(InstantCommand(lambda: self.coral.liftCoral(-self.coralLiftSpeed, 0))).onFalse(InstantCommand(lambda: self.coral.stopLiftCoral())) 

        #depositing coral
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).whileTrue(InstantCommand(lambda: self.coral.runDepositCoral(self.coralDepositSpeed))).onFalse(InstantCommand(lambda: self.coral.stopDepositCoral())) 

        # CLIMBER
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).whileTrue(InstantCommand(lambda: self.coral.liftCoral)).onFalse(InstantCommand(lambda: self.coral.stop())) 

        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftStick).whileTrue(InstantCommand(lambda: self.climber.runClimbingMotor1(self.climberSpeed))).onFalse(InstantCommand(lambda: self.climber.stopClimbingMotor1()))
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightStick).whileTrue(InstantCommand(lambda: self.climber.runClimbingMotor2(-self.climberSpeed))).onFalse(InstantCommand(lambda: self.climber.stopClimbingMotor2()))
        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kLeftBumper).whileTrue(InstantCommand(lambda: self.climber.runClimbingMotors(self.climberSpeed))).onFalse(InstantCommand(lambda: self.climber.stopClimbingMotors()))

        # ALGAE
        
        # Old Algae code

        #taking in Algae (running motors backwards)
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftStick).whileTrue(InstantCommand(lambda: self.algae.runAlgae(-self.algaeIntakeSpeed))).onFalse(InstantCommand(lambda: self.algae.stopAlgae()))

        #spitting the algae out
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightStick).whileTrue(InstantCommand(lambda: self.algae.runAlgae(self.algaeIntakeSpeed))).onFalse(InstantCommand(lambda: self.algae.stopAlgae()))

        #Lift algae
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).whileTrue(InstantCommand(lambda: self.algae.runLiftAlgae(self.algaeLiftSpeed))).onFalse(InstantCommand(lambda: self.algae.stopLiftAlgae()))
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).whileTrue(InstantCommand(lambda: self.algae.runLiftAlgae(-self.algaeLiftSpeed))).onFalse(InstantCommand(lambda: self.algae.stopLiftAlgae()))

        
        # might have to change/create more variables
        # Could also create new file and copy from SwerveJoystickCmd.py
        self.xSpeed = self.operatorController.getLeftX() * DrivingConstants.drivingSpeedLimiter #self.drivingLimiter#* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ySpeed = self.operatorController.getLeftY() * DrivingConstants.drivingSpeedLimiter #self.drivingLimiter #* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.zRotation = self.operatorController.getRightX() * -1 * DrivingConstants.rotationSpeedLimiter
        self.xSpeed = wpimath.applyDeadband(self.xSpeed, OIConstants.deadzone)
        self.ySpeed = wpimath.applyDeadband(self.ySpeed, OIConstants.deadzone)
        self.zRotation = wpimath.applyDeadband(self.zRotation, OIConstants.deadzone)
        
        

        # Test Buttons

        # Climbers
        # Up-Down Separate Climbers
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).wileTrue(InstantCommand(lambda: self.intake.runIntake(-0.5))).onFalse(InstantCommand(lambda: self.intake.stopIntake()))
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).whileTrue(InstantCommand(lambda: self.midstage.runMidstage(-0.75))).onFalse(InstantCommand(lambda: self.midstage.stopMidstage()))

        # # Toggle Climbers
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftStick).whileTrue(InstantCommand(lambda: self.climber.runLeftClimbingMotor(0.75))).onFalse(InstantCommand(lambda: self.climber.stopLeftClimbingMotor()))
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightStick).whileTrue(InstantCommand(lambda: self.climber.runRightClimbingMotor(-0.75))).onFalse(InstantCommand(lambda: self.climber.stopRightClimbingMotor()))
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).whileTrue(InstantCommand(lambda: self.climber.runLeftClimbingMotor(-0.75))).onFalse(InstantCommand(lambda: self.climber.stopLeftClimbingMotor()))
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).whileTrue(InstantCommand(lambda: self.climber.runRightClimbingMotor(0.75))).onFalse(InstantCommand(lambda: self.climber.stopRightClimbingMotor()))
