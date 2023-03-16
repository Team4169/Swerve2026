import wpilib
from wpilib.interfaces import GenericHID
import ntcore

import rev, ctre

import commands2
import commands2.button

import constants

# from commands.complexauto import ComplexAuto
# from commands.drivedistance import DriveDistance
from commands.defaultdrive import DefaultDrive
# from commands.halvedrivespeed import HalveDriveSpeed
from commands.coneToBalanceAuto import coneToBalanceAuto
from commands.cubeToBalanceAuto import cubeToBalanceAuto
# from commands.moveForwardToVisionTarget import MoveForwardToVisionTarget
# from commands.centerRobotToTarget import CenterRobotToTarget
# from commands.lucautocommandInverted import LucAutoCommand2
# from commands.newPath import newPath
# from commands.newPathInverted import newPathInverted
# from commands.zeroBall import zeroBall
#
# from commands.SnowVeyerCommands.DropOff import dropOff
# from commands.SnowVeyerCommands.PickUp import pickUp
# from commands.SnowVeyerCommands.intake import Intake
# from commands.SnowVeyerCommands.outtake import Outtake
#
#
# from commands.climbingCommands.moveRotateArm import MoveRotateArm
# from commands.climbingCommands.liftArmToTop import LiftArmToTop
# from commands.climbingCommands.moveLiftArm import MoveLiftArm
# from commands.climbingCommands.moveLiftArmToLimitSwitch import MoveLiftArmToLimitSwitch
# from commands.climbingCommands.moveLiftArmPastLocation import MoveLiftArmPastLocation
# from commands.climbingCommands.liftArmToTop import LiftArmToTop
# from commands.climbingCommands.coastRotateArm import coastRotateArm

from commands.doNothing import DoNothing

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem
# #from subsystems.snowveyorsubsystem import SnowveyorSubsystem
# #from subsystems.climbingsubsystem import ClimbingSubsystem

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.driverController = wpilib.XboxController(constants.kDriverControllerPort)
        self.operatorController = wpilib.XboxController(constants.kArmControllerPort)

        self.leftTalon = ctre.WPI_TalonSRX(constants.leftTalon)
        self.leftTalon2 = ctre.WPI_TalonSRX(constants.leftTalon2)
        self.rightTalon = ctre.WPI_TalonSRX(constants.rightTalon)
        self.rightTalon2 = ctre.WPI_TalonSRX(constants.rightTalon2)

        #Arm motor controllers
        self.grabbingArm = rev.CANSparkMax(constants.grabbingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushed) #type: rev._rev.CANSparkMaxLowLevel.MotorType
        self.extendingArm = rev.CANSparkMax(constants.extendingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rotatingArm = rev.CANSparkMax(constants.rotatingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        #Arm motor encoders
        self.grabbingArmEncoder = wpilib.Counter(wpilib._wpilib.DigitalInput(constants.grabbingArmEncoderPort))
        self.extendingArmEncoder = self.extendingArm.getEncoder()
        self.rotatingArmEncoder = self.rotatingArm.getEncoder()
        # self.neoMotor = rev.CANSparkMax(constants.neoMotor, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        # self.table = NetworkTables.getTable("limelight")
        # tx = self.table.getNumber('tx', None)
        # ty = self.table.getNumber('ty', None)
        # ta = self.table.getNumber('ta', None)
        # ts = self.table.getNumber('ts', None)
        # tv = self.table.getNumber('tv', None)

        # self.liftArm = rev.CANSparkMax(constants.liftArm, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        # self.rotateArm = rev.CANSparkMax(constants.rotateArm, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        
        #self.liftArm.setIdleMode(self.liftArm.IdleMode(1))
        #self.rotateArm.setIdleMode(self.rotateArm.IdleMode(1))

        #self.liftArm.setInverted(True)

        # self.rotateEncoder = self.rotateArm.getEncoder()
        # self.liftEncoder = self.liftArm.getEncoder(rev.SparkMaxRelativeEncoder.Type.kQuadrature)

        #^ forward is grabbing, we may need to switch this
        self.grabbingArmOpenLimitSwitch = self.grabbingArm.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.grabbingArmClosedLimitSwitch = self.grabbingArm.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.extendingArmMaxLimitSwitch = self.extendingArm.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.extendingArmMinLimitSwitch = self.extendingArm.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.rotatingArmMaxLimitSwitch = self.rotatingArm.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.rotatingArmMinLimitSwitch = self.rotatingArm.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)

        self.coastBool=False

        # The robot's subsystems

        self.drive = DriveSubsystem(leftTalon=self.leftTalon,
                                    leftTalon2=self.leftTalon2,
                                    rightTalon=self.rightTalon,
                                    rightTalon2=self.rightTalon2) #.drive
        
        self.arm = ArmSubsystem(grabbingArm=self.grabbingArm,
                                extendingArm=self.extendingArm,
                                rotatingArm=self.rotatingArm,
                                grabbingArmLimitSwitchOpen=self.grabbingArmOpenLimitSwitch,
                                grabbingArmLimitSwitchClosed=self.grabbingArmClosedLimitSwitch,
                                extendingArmLimitSwitchMin=self.extendingArmMinLimitSwitch,
                                extendingArmLimitSwitchMax=self.extendingArmMaxLimitSwitch,
                                rotatingArmLimitSwitchMin=self.rotatingArmMinLimitSwitch,
                                rotatingArmLimitSwitchMax=self.rotatingArmMaxLimitSwitch,
                                grabbingArmEncoder=self.grabbingArmEncoder,
                                extendingArmEncoder=self.extendingArmEncoder,
                                rotatingArmEncoder=self.rotatingArmEncoder
                                )

        inst = ntcore.NetworkTableInstance.getDefault()
        self.sd = inst.getTable("SmartDashboard")
        self.posInitSD = self.sd.getDoubleTopic("posInit").subscribe(1)
        self.posEndSD = self.sd.getDoubleTopic("posEnd").subscribe(1)
        self.targetSD = self.sd.getDoubleTopic("Target").subscribe(1)




        # self.snowveyor = SnowveyorSubsystem(intake=self.intake,
        #                                     outtake=self.outtake,
        #                                     snowveyor=self.snowveyor)
        #
        # self.climb = ClimbingSubsystem(liftArm=self.liftArm,
        #                                rotateArm=self.rotateArm,
        #                                liftEncoder=self.liftEncoder,
        #                                rotateEncoder=self.rotateEncoder,
        #                                liftArmUpLimitSwitch=self.liftArmUpLimitSwitch,
        #                                liftArmDownLimitSwitch=self.liftArmDownLimitSwitch,
        #                                rotateArmBackLimitSwitch=self.rotateArmBackLimitSwitch,
        #                                rotateArmRobotLimitSwitch=self.rotateArmRobotLimitSwitch)

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        # self.simpleAuto = DriveDistance(
        #     constants.kAutoDriveDistanceInches, constants.kAutoDriveSpeed, self.drive
        # )

        # A complex auto routine that drives forward, and then drives backward.
        # self.complexAuto = ComplexAuto(self.drive)

        # A complex auto routine that drives forward, and then drives backward.
        #  self.lucAutoCommand = LucAutoCommand(self.drive, self.snowveyor)
        self.coneToBalance =coneToBalanceAuto(self.drive)
        self.cubeToBalance = cubeToBalanceAuto(self.drive)
        # self.moveForwardToVisionTarget = MoveForwardToVisionTarget(self.drive, self.neoMotor)
        # self.centerRobotToTarget = CenterRobotToTarget(self.drive, self.neoMotor)
        # self.lucAutoCommand2 = LucAutoCommand2(self.drive, self.snowveyor)
        # #simpler auto routine that drives to the second ball and places 2 into the smaller hub
        # self.newPath = newPath(self.drive, self.snowveyor)
        # self.newPathInverted = newPathInverted(self.drive, self.snowveyor)
        # self.zeroBall = zeroBall(self.drive, self.snowveyor)
        # Chooser
        self.chooser = wpilib.SendableChooser()
        #
        # # Add commands to the autonomous command chooser
        # # self.chooser.setDefaultOption("Complex Auto", self.complexAuto)
        self.chooser.setDefaultOption("Cube to Balance", self.cubeToBalance)
        # # self.chooser.addOption("Simple Auto", self.simpleAuto)
        self.chooser.addOption("Cone to Balance", self.coneToBalance)
        # self.chooser.addOption("Luc AutoInverted", self.lucAutoCommand2)
        # self.chooser.addOption("SimplePath", self.newPath)
        # self.chooser.addOption("SimplePathInverted", self.newPathInverted)
        # self.chooser.addOption("zeroBall", self.zeroBall)
        # # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomousff", self.chooser)

        # self.configureButtonBindings()

        # set up default drive command
        # self.drive.setDefaultCommand(
        #     DefaultDrive(
        #         self.drive,
        #         lambda: -self.driverController.getRightY(),
        #         lambda: self.driverController.getLeftY(),
        #     )
        # )
        #commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kY).whenPressed(
        #    MoveForwardToVisionTarget(self.drive, self.neoMotor))
        #commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kB).whenPressed(
        #    CenterRobotToTarget(self.drive, self.neoMotor))

    # def bindClimbMode(self):
    #     """
    #     Use this method to define your button->command mappings. Buttons can be created by
    #     instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
    #     and then passing it to a JoystickButton.
    #     """
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).whenHeld(
    #         MoveLiftArm(-.5, self.climb)
    #     )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whenHeld(
    #         MoveLiftArm(1, self.climb)
    #     )
    #     # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whenHeld(
    #     #     MoveLiftArm(.5, self.climb)
    #     # )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).whenHeld(
    #         MoveLiftArm(-1, self.climb)
    #     )
    #     # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whenHeld(
    #     #     MoveLiftArm(.5, self.climb)
    #     # )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).whenHeld(
    #         MoveRotateArm(.3, self.climb)
    #     )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).whenHeld(
    #         MoveRotateArm(-.3, self.climb)
    #     )
    #     commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kA).whenHeld(
    #        coastRotateArm(self.coastBool, self.climb)
    #     )
    #
    # def unbindClimbMode(self):
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kLeftBumper).whenHeld(
    #         DoNothing()
    #     )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whenHeld(
    #         DoNothing()
    #     )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kA).whenHeld(
    #         DoNothing()
    #     )
    #     # commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kY).whenHeld(
    #     #     DoNothing()
    #     # )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kX).whenHeld(
    #         DoNothing()
    #     )
    #     commands2.button.JoystickButton(self.operatorController, wpilib.XboxController.Button.kB).whenHeld(
    #         DoNothing()
    #     )
    #     commands2.button.JoystickButton(self.driverController, wpilib.XboxController.Button.kA).whenPressed(
    #         DoNothing()
    #     )
    #
    #
    #
    def getAutonomousCommand(self) -> commands2.Command:
        self.target = self.targetSD.get()-1
        return self.chooser.getSelected()
    def getDistanceAuto(self,cone:bool):
        y = constants.startPos[self.posInitSD.get()-1] + constants.cubeToConeDistance
        b = constants.endPos[self.posEndSD.get()-1] + constants.cubeToConeDistance
        if cone:
            y+= constants.cubeToConeDistance
            b+= constants.cubeToConeDistance

        a = constants.balanceDistance
        driveDistance = math.sqrt((math.abs(y - b) ** 2 + a ** 2))
        return driveDistance
    def getAngleAuto(self,cone:bool):
        y = constants.startPos[self.posInitSD.get()] + constants.cubeToConeDistance
        b = constants.endPos[self.posEndSD.get()] + constants.cubeToConeDistance
        if cone:
            y += constants.cubeToConeDistance
            b += constants.cubeToConeDistance
        a = constants.balanceDistance
        angle = math.arctan(math.abs(y - b) / a)
        return angle
