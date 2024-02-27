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
from pathplannerlib.path import PathPlannerPath #.path
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

        #Arm motor controllers
        # self.grabbingArm = rev.CANSparkMax(RobotConstants.grabbingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushed) #type: rev._rev.CANSparkMaxLowLevel.MotorType
        # self.extendingArm = rev.CANSparkMax(RobotConstants.extendingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        # self.rotatingArm = rev.CANSparkMax(RobotConstants.rotatingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        #Arm motor encoders
        # self.grabbingArmEncoder = wpilib.Counter(wpilib._wpilib.DigitalInput(RobotConstants.grabbingArmEncoderPort))
        # self.extendingArmEncoder = self.extendingArm.getEncoder()
        # self.rotatingArmEncoder = self.rotatingArm.getEncoder()

        #^ forward is grabbing, we may need to switch this
        # self.grabbingArmOpenLimitSwitch = self.grabbingArm.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # self.grabbingArmClosedLimitSwitch = self.grabbingArm.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # self.extendingArmMaxLimitSwitch = self.extendingArm.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # self.extendingArmMinLimitSwitch = self.extendingArm.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # self.rotatingArmMaxLimitSwitch = self.rotatingArm.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # self.rotatingArmMinLimitSwitch = self.rotatingArm.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)

        # The robot's subsystems
        self.swerve = SwerveSubsystem()

        self.intake = IntakeSubsystem()
        self.midstage = MidstageSubsystem()
        self.climber = ClimbingSubsystem()
        self.shooter = ShooterSubsystem()

        # self.shooter = ShooterSubsystem()

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
       
        self.configureButtonBindings()

        self.chooser = wpilib.SendableChooser()

        # self.chooser.setDefaultOption("cubeAuto", self.cubeToBalance )
        # self.chooser.addOption("simple auto", self.simpleAuto)

        # # Put the chooser on the dashboard
        # self.shuffle = wpilib.SmartDashboard
        # self.shuffle.putData("Autonomousff", self.chooser)
        # self.shuffle.putData("moveTest", self.moveTest)
        # self.shuffle.putData("dropOffAngle", self.dropOffAngle)
        # self.shuffle.putData("dropOffExtend", self.dropOffExtend)
        # self.shuffle.putData("dropObject", self.dropObject)

        # self.camera = photonvision.PhotonCamera("Microsoft_LifeCam_HD-3000")
    def getAutonomousCommand(self) -> commands2.Command:
        """Returns the autonomous command to run"""
       
        # # #make the robot go to an april tag
        # # self.angle = 0 #make sure to use a radian angle here
        # # self.distance = 0
        # # self.aprilTagTrajectory = TrajectoryGenerator.generateTrajectory(
        # #     # ? initial location and rotation
        # #     Pose2d(0,0, Rotation2d(0)),
        # #     [
        # #         # ? points we want to hit
        # #         #april tag infront of the robot is 0 degrees
        # #         #this is in radians
        # #         Translation2d(self.distance*math.sin(self.angle * 180 / math.pi), self.distance*math.cos(self.angle * 180 / math.pi)),
                
        # #     ],
        # #     # ? final location and rotation
        # #     Pose2d(0, 0, Rotation2d(180)),
        # #     self.trajectoryConfig
        # # )

        self.move1module = move1module(self.swerve)
        self.move2motors = move2motors(self.swerve)
        self.move4modules = move4modules(self.swerve)
        self.MoveInACircle = MoveInACircle(self.swerve)
        # self.sCurve = sCurve(self.swerve).getCommand()

        #^^Added this today (1/11)
        # path = PathPlannerPath.fromPathFile('Line')
        # Create a path following command using AutoBuilder. This will also trigger event markers.

        return PathPlannerAuto('GetOutOfTheWay1')

        # return AutoBuilder.followPath(path)

        #optimize clip https://youtu.be/0Xi9yb1IMyA?t=225

    def configureButtonBindings(self):
        pass
        #  (
        # JoystickButton(self.driverController, XboxController.Button.kStart).whenPressed(lambda: self.swerve.zeroHeading())
        # )
        # (
        #JoystickButton(self.driverController, XboxController.Button.kY).whenHeld(lockWheels(self.swerve)).whenReleased(lambda: self.swerve.unlockWheels())
        # )

        # self.driverController.start().onTrue(commands2.cmd.run(lambda: self.swerve.zeroHeading()))

        # self.driverController.Y().whileActiveContinous(commands2.cmd.run(lambda: self.swerve.lockWheels()))
        # self.driverController.Y().onFalse(commands2.cmd.run(lambda: self.swerve.unlockWheels()))
        
        
        #Old way of Assigning Buttons
            
#             commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kStart).onTrue(InstantCommand(lambda: self.swerve.zeroHeading()))
#             # commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kBack).whileTrue(InstantCommand(lambda: self.swerve.lockWheels())).onFalse(lambda: self.swerve.unlockWheels())

#             #*run intake and midtsage to bring the ring into the robot
#             commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whileTrue(InstantCommand(lambda: self.intake.runIntake(0.25))).onFalse(lambda: self.intake.stopIntake())
#             commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whileTrue(InstantCommand(lambda: self.midstage.runMidstage(0.25))).onFalse(lambda: self.midstage.stopMidstage())
#             #run the intake and midstage to eject the ring
#             commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).whileTrue(InstantCommand(lambda: self.intake.runIntake(-0.25))).onFalse(lambda: self.intake.stopIntake())
#             commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).whileTrue(InstantCommand(lambda: self.midstage.runMidstage(-0.25))).onFalse(lambda: self.midstage.stopMidstage())                       
            
#             #*Shooter launch
#             commands2.button.JoystickButton(self.operatorController, XboxController.Button.kX).whileTrue(InstantCommand(lambda: self.shooter.runShooter())).onFalse(lambda: self.shooter.stopShooter())

#             #*Backup for if the rotating shooter doesn't work
#             commands2.button.JoystickButton(self.operatorController, XboxController.Button.kStart).whileTrue(InstantCommand(lambda: self.shooter.setShooterAngle(Rotation2d(constants.RobotConstants.backupShooterAngle)))).onFalse(lambda: self.shooter.stopRotating())
#             #these are the backups for the backups
#             #if the setShooter angle doesn't work and the backupshooterangle doesn't work
#             #then the operator would rotate the shooter angle manually    
            
#             commands2.button.JoystickButton(self.operatorController, XboxController.Button.kRightStick).whileTrue(InstantCommand(lambda: self.shooter.rotateManually(0.1))).onFalse(lambda: self.shooter.stopRotating())
#             commands2.button.JoystickButton(self.operatorController, XboxController.Button.kLeftStick).whileTrue(InstantCommand(lambda: self.shooter.rotateManually(-0.1))).onFalse(lambda: self.shooter.stopRotating())
#  #!                                                                                    ^^^kback otherwise^^^
           
    def rotateToSpeaker(self, rotation):
        self.rotateToSpeakerCommand = rotateToSpeakerCommand(self.swerve, rotation)
        self.rotateToSpeakerCommand.schedule()
