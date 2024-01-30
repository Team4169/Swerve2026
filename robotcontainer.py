import wpilib, wpimath
from wpilib.interfaces import GenericHID
from wpimath.geometry import Pose2d, Rotation2d, Translation2d



import ntcore, rev, commands2
import commands2.cmd
import commands2.button


import constants
from constants import AutoConstants, OIConstants, RobotConstants

from commands.TeleopCommands.SwerveJoystickCmd import SwerveJoystickCmd

from commands2.button import JoystickButton, CommandXboxController
from wpilib import XboxController, Joystick

from subsystems.armsubsystem import ArmSubsystem 
from subsystems.swervesubsystem import SwerveSubsystem
import math
# import photonvision

from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath #.path
from pathplannerlib.auto import AutoBuilder #.auto

from commands.testcommands.move1module import move1module
from commands.testcommands.move2motors import move2motors
from commands.testcommands.move4modules import move4modules
from commands.testcommands.MoveInACircle import MoveInACircle
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

        self.swerve.setDefaultCommand(SwerveJoystickCmd(
                swerve=self.swerve,
                driverController = self.driverController
            ))
        
        #^^Added this today (1/11)
        NamedCommands.registerCommand("resetOdometry",
            commands2.InstantCommand(lambda:self.swerve.resetOdometry(self.trajectory.initialPose()))
        )
        NamedCommands.registerCommand("stopModules",
            commands2.InstantCommand(lambda:self.swerve.stopModules())
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
        # self.oval = PathPlannerAuto("ovalAuton")
        self.sCurve = sCurve(self.swerve).getCommand()

        #^^Added this today (1/11)
        path = PathPlannerPath.fromPathFile('Oval')
        # Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)

        #optimize clip https://youtu.be/0Xi9yb1IMyA?t=225

    def configureButtonBindings(self):
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
            commands2.button.JoystickButton(self.driverController, XboxController.Button.kStart).onTrue(InstantCommand(lambda: self.swerve.zeroHeading()))
            commands2.button.JoystickButton(self.driverController, XboxController.Button.kX).whileTrue(InstantCommand(lambda: self.swerve.lockWheels())).onFalse(lambda: self.swerve.unlockWheels())
            
            # Joystick.button(self.driverController, )
