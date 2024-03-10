import wpilib, wpimath
from wpilib.interfaces import GenericHID
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

import ntcore, rev, commands2
import commands2.cmd
import commands2.button

import constants
from constants import AutoConstants, OIConstants, RobotConstants

from commands.TeleopCommands.SwerveJoystickCmd import SwerveJoystickCmd
from commands2.button.trigger import Trigger
from commands2.button import JoystickButton, CommandXboxController
from wpilib import XboxController, Joystick

from subsystems.armsubsystem import ArmSubsystem 
from subsystems.swervesubsystem import SwerveSubsystem

from subsystems.intakeSubsystem import IntakeSubsystem
from subsystems.midstageSubsystem import MidstageSubsystem
from subsystems.climbingSubsystem import ClimbingSubsystem
from subsystems.ShooterSubsystem import ShooterSubsystem

import math
# import photonvision
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath, PathConstraints
from pathplannerlib.auto import AutoBuilder #.auto


from commands.testcommands.move1module import move1module
from commands.testcommands.move2motors import move2motors
from commands.testcommands.move4modules import move4modules
from commands.testcommands.MoveInACircle import MoveInACircle

from commands.testcommands.rotateToSpeakerCommand import rotateToSpeakerCommand

from commands.AutonCommands.sCurve import sCurve
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
        self.swerve = SwerveSubsystem()

        self.intake = IntakeSubsystem()
        self.midstage = MidstageSubsystem()
        # self.climber = ClimbingSubsystem()
        self.shooter = ShooterSubsystem()

        self.swerve.setDefaultCommand(SwerveJoystickCmd(
                swerve=self.swerve,
                driverController = self.driverController
            ))
        
        #^^Added this today (1/11)
        #commands for pathplanner
        NamedCommands.registerCommand("resetOdometry",
            commands2.InstantCommand(lambda:self.swerve.resetOdometry(self.trajectory.initialPose()))
        )
        NamedCommands.registerCommand("stopModules",
            commands2.InstantCommand(lambda:self.swerve.stopModules())
        )
        NamedCommands.registerCommand("pickupRing",
            commands2.InstantCommand(lambda:self.intake.runIntake(0.50))
        )
        NamedCommands.registerCommand("stopIntake",
            commands2.InstantCommand(lambda:self.intake.stopIntake())
        )
        NamedCommands.registerCommand("midstageRing",
            commands2.InstantCommand(lambda:self.midstage.runMidstage(0.50))
        )
        NamedCommands.registerCommand("stopMidstage",
            commands2.InstantCommand(lambda:self.midstage.stopMidstage())
        )
        NamedCommands.registerCommand("setShooterAngle",
            commands2.InstantCommand(lambda:self.shooter.setShooterAngle(self.shooter.getShooterAngle()))
        )
        NamedCommands.registerCommand("stopShooter",
            commands2.InstantCommand(lambda:self.midstage.stopShooter())
        )
        NamedCommands.registerCommand("shootRing",
            commands2.InstantCommand(lambda:self.shooter.runShooter())
        )

       
        self.configureButtonBindings()

        # sendable chooser
        self.chooser = wpilib.SendableChooser()

        # self.RF1B1B2 = "RF1B1B2" # Auto 1 
        # self.RB5B4B3 = "RB5B4B3" #Auto 2
        # self.RF3F2 = "RF3F2" #Auto 3 (4 1, 3 3 , 3 4, 3 5)
        # self.RF3B3B2 = "RF3B3B2" # Auto 4
        # self.RB5B4 = "RB5B4" # Auto 5
        # self.RF1B1F2 = "RF1B1F2" # Auto 6
       # self.RB5B4_2 = "RB5B4_2" # Auto 7
        self.GetOutOfTheWay1 = "GetOutOfTheWay1"
        # self.RF1F2F3 = "RF1F2F3" #Auto 8 
        # self.mirrorTest = "mirrorTest"

        # self.chooser.setDefaultOption("RF1B1B2", self.RF1B1B2) # Auto 1
        # self.chooser.addOption("RB5B4B3", self.RB5B4B3) #Auto 2
        # self.chooser.addOption("RF3F2", self.RF3F2) #Auto 3
        # self.chooser.addOption("RF3B3B2", self.RF3B3B2) # Auto 4
        # self.chooser.addOption("RB5B4", self.RB5B4)# Auto 5
        # self.chooser.addOption("RF1B1F2", self.RF1B1F2) # Auto 6
       # self.chooser.addOption("RB5B4_2", self.RB5B4_2) # Auto 7
        self.chooser.addOption("GetOutOfTheWay1", self.GetOutOfTheWay1)
        # self.chooser.addOption("RF1F2F3", self.RF1F2F3) # Auto 8 
        # self.chooser.addOption("mirrorTest", self.mirrorTest) 

        # # Put the chooser on the dashboard
        self.shuffle = wpilib.SmartDashboard
        self.shuffle.putData("Autonomousff", self.chooser)

        self.network_tables = ntcore.NetworkTableInstance.getDefault()
        self.datatable = self.network_tables.getTable("datatable")

        # self.camera = photonvision.PhotonCamera("Microsoft_LifeCam_HD-3000")
    
    '''
    def getObjectDetectionPathfinding(self) -> commands2.Command:
        """Returns the pathfinding command from position"""

        changeHorizontal = datatable.getEntry("objHorizontal").getValue()
        changeDistance = datatable.getEntry("objDistance").getValue()

        changeRot = math.atan2(changeHorizontal, changeDistance)
        self.targetPose = Pose2d(changeDistance, changeHorizontal, Rotation2d.fromRadians(changeRot))

        pathfindingCommand = AutoBuilder.pathfindToPose(
            self.targetPose,
            AutoConstants.constraints,
            goal_end_vel=0.0,
            rotation_delay_distance=0.0
        )
        
        autoCommand = PathPlannerAuto(pathfindingCommand)
        if autoCommand:
            autoCommand.schedule()
    '''

    def getAutonomousCommand(self) -> commands2.Command:
        """Returns the autonomous command to run"""

        return PathPlannerAuto(self.chooser.getSelected())

    def configureButtonBindings(self):
        commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kStart).onTrue(InstantCommand(lambda: self.swerve.zeroHeading()))
        # commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kY).onTrue(InstantCommand(lambda: self.getObjectDetectionPathfinding()))
        # commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kBack).whileTrue(InstantCommand(lambda: self.swerve.lockWheels())).onFalse(lambda: self.swerve.unlockWheels())

        #*run intake and midstage to bring the ring into the robot
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whileTrue(InstantCommand(lambda: self.intake.runIntake(-0.5))).onFalse(InstantCommand(lambda: self.intake.stopIntake()))
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whileTrue(InstantCommand(lambda: self.midstage.runMidstage(-0.5))).onFalse(InstantCommand(lambda: self.midstage.stopMidstage()))
        #run the intake and midstage to eject the ring
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).whileTrue(InstantCommand(lambda: self.intake.runIntake(0.5))).onFalse(InstantCommand(lambda: self.intake.stopIntake()))
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).whileTrue(InstantCommand(lambda: self.midstage.runMidstage(0.5))).onFalse(InstantCommand(lambda: self.midstage.stopMidstage()))                       
            
        #*Shooter launch
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).whileTrue(InstantCommand(lambda: self.shooter.runShooter())).onFalse(InstantCommand(lambda: self.shooter.stopShooter()))
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).whileTrue(InstantCommand(lambda: self.midstage.runMidstage(-0.75))).onFalse(InstantCommand(lambda: self.midstage.stopMidstage()))

        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).whileTrue(InstantCommand(lambda: self.intake.runIntake(-0.5))).onFalse(InstantCommand(lambda: self.intake.stopIntake()))
        commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).whileTrue(InstantCommand(lambda: self.midstage.runMidstage(-0.75))).onFalse(InstantCommand(lambda: self.midstage.stopMidstage()))






        # commands2.button.JoystickButton(self.operatorCon/troller, wpilib.XboxController.Button.kLeftBumper).whileTrue(InstantCommand(lambda: self.climber.runLeftClimbingMotor(0.4))).whileFalse(InstantCommand(lambda: self.climber.runLeftClimbingMotor(-0.4)))
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kRightBumper).whileTrue(InstantCommand(lambda: self.climber.runRightClimbingMotor(0.4))).onFalse(InstantCommand(lambda: self.climber.runRightClimbingMotor(-0.4)))

        # #*Backup for if the rotating shooter doesn't work 
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kStart).whileTrue(InstantCommand(lambda: self.shooter.setShooterAngle(float(constants.RobotConstants.backupShooterAngle)))).onFalse(lambda: self.shooter.stopRotating())
        # #these are the backups for the backups#                                                                                                                 ^^^^   this was a Rotation2d before^^^
        # #if the setShooter angle doesn't work and the backupshooterangle doesn't work
        # #then the operator would rotate the shooter angle manually   
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).whileTrue(InstantCommand(lambda: self.shooter.setShooterAngle(self.shooter.getShooterAngle()))).onFalse(lambda: self.shooter.stopRotating())
        # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).whileTrue(InstantCommand(lambda: self.rotateToSpeaker()))

        # #  (
        # JoystickButton(self.driverController, XboxController.Button.kStart).whenPressed(lambda: self.swerve.zeroHeading())
        # )
        # (
        #JoystickButton(self.driverController, XboxController.Button.kY).whenHeld(lockWheels(self.swerve)).whenReleased(lambda: self.swerve.unlockWheels())
        # )

        # self.driverController.start().onTrue(commands2.cmd.run(lambda: self.swerve.zeroHeading()))

        # self.driverController.Y().whileActiveContinous(commands2.cmd.run(lambda: self.swerve.lockWheels()))
        # self.driverController.Y().onFalse(commands2.cmd.run(lambda: self.swerve.unlockWheels()))


    def rotateToSpeaker(self):
        self.jetson1rotation = self.camera_tables.getEntry("r1").getValue()
        self.jetson2rotation = self.camera_tables.getEntry("r2").getValue()
        self.jetson1X = self.camera_tables.getEntry("x1").getValue()
        self.jetson2X = self.camera_tables.getEntry("x2").getValue()
        self.jetson1Y = self.camera_tables.getEntry("y1").getValue()
        self.jetson2Y = self.camera_tables.getEntry("y2").getValue()
        self.jetson1weight = self.camera_tables.getEntry("w1").getValue()
        self.jetson2weight = self.camera_tables.getEntry("w2").getValue()

        self.rotationAve = (self.jetson1rotation + self.jetson2rotation) /2
        self.xAve = (self.jetson1X + self.jetson2X) /2
        self.yAve = (self.jetson1Y + self.jetson2Y) /2
        self.weightAve = (self.jetson1weight + self.jetson2weight) / 2
        
        self.xDistance = self.RobotConstants.speakerToCenterOfFieldX - self.xAve #8.3m 
        self.yDistance = self.RobotConstants.heightOfField - self.yAve #1.45m
        self.distanceToShooter = math.sqrt(self.xDistance**2 + self.yDistance**2)

        rotation = Rotation2d(math.acos(self.xDistance/self.distanceToShooter)) 

        self.rotateToSpeakerCommand = rotateToSpeakerCommand(self.swerve, rotation)
        #print("running rotate to speaker")
        self.rotateToSpeakerCommand.schedule()
        
