import commands2
import wpilib
import wpilib.drive
import ctre
import constants
from networktables import NetworkTables
import wpimath.controller

class SnowveyorSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        # smartdashboard
        self.sd = NetworkTables.getTable("SmartDashboard")

        # Snowveyor
        self.intake = ctre.WPI_VictorSPX(constants.intake)
        self.outtake = ctre.WPI_VictorSPX(constants.outtake)
        self.snowveyor = wpilib.drive.DifferentialDrive(self.intake, self.outtake)

    def arcadeDrive(self, fwd: float, rot: float) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(fwd, rot)


    def getAverageEncoderTicks(self) -> float:
        """Gets the average distance of the TWO encoders."""
        self.sd.putValue("Left Encoder Value", self.left1.getSelectedSensorPosition())
        self.sd.putValue("Right Encoder Value", self.right2.getSelectedSensorPosition())
        return (self.left1.getSelectedSensorPosition() + self.right2.getSelectedSensorPosition()) / -2.0

    def setIntake(self, speed: float):
        self.intake.set(speed)

    def setOuttake(self, speed: float):
        self.outtake.set(speed)
        self.intake.set(speed)
