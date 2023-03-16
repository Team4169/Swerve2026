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

from commands.doNothing import DoNothing

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem

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

        self.coneToBalance =coneToBalanceAuto(self.drive)
        self.cubeToBalance = cubeToBalanceAuto(self.drive)
        #chooser
        self.chooser = wpilib.SendableChooser()

        self.chooser.setDefaultOption("Cube to Balance", self.cubeToBalance)
        self.chooser.addOption("Cone to Balance", self.coneToBalance)

        # # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomousff", self.chooser)


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
