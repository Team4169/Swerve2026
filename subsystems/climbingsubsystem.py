import commands2
import wpilib
import wpilib.drive
import ctre
import constants
from networktables import NetworkTables
import wpimath.controller
import rev


class ClimbingSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.liftArm = rev.CANSparkMax(constants.liftArm, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.rotateArm = rev.CANSparkMax(constants.rotateArm, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.liftArm.setIdleMode(self.liftArm.IdleMode(1))
        self.rotateArm.setIdleMode(self.rotateArm.IdleMode(1))

        self.liftArm.setInverted(True)

        self.rotateEncoder = self.rotateArm.getEncoder()
        self.liftEncoder = self.liftArm.getEncoder(rev.SparkMaxRelativeEncoder.Type.kQuadrature)

        self.liftArmUpLimitSwitch = wpilib.DigitalInput(0)
        self.rotateArmBackLimitSwitch = wpilib.DigitalInput(2)

        # smartdashboard
        self.sd = NetworkTables.getTable("SmartDashboard")


    def resetEncoders(self) -> None:
        pass
        # self.left1.setSelectedSensorPosition(0, 0, 10)
        # self.right2.setSelectedSensorPosition(0, 0, 10)
        """Resets the drive encoders to currently read a position of 0."""

    def getLiftArmEncoderDistance(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return self.liftEncoder.getPosition()

    def getRotateArmEncoderDistance(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return self.rotateEncoder.getPosition()

    def setRotateArm(self, speed):
        self.rotateArm.set(speed)

    def setLiftArm(self, speed):
        self.liftArm.set(speed)
