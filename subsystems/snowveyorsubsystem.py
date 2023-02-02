import commands2
import wpilib
import wpilib.drive
import ctre
import constants
import ntcore
import wpimath.controller

class SnowveyorSubsystem(commands2.SubsystemBase):
    def __init__(self, intake, outtake, snowveyor) -> None:
        super().__init__()

        # smartdashboard
        # # self.sd = NetworkTables.getTable("SmartDashboard")

        self.intake = intake
        self.outtake = outtake
        self.snowveyor = snowveyor

    def tankDrive(self, intakespeed: float, outtakespeed: float) -> None:
        """
        Controls snowveyor with tankDrive. Not sure why we are using this implementation - Noah
        """
        self.snowveyor.tankDrive(intakespeed, outtakespeed)


    def getAverageEncoderTicks(self) -> float:
        """Gets the average distance of the TWO encoders."""
        # self.sd.putValue("Left Encoder Value", self.left1.getSelectedSensorPosition())
        # self.sd.putValue("Right Encoder Value", self.right2.getSelectedSensorPosition())
        return (self.left1.getSelectedSensorPosition() + self.right2.getSelectedSensorPosition()) / -2.0

    def setIntake(self, speed: float):
        self.intake.set(speed)

    def setOuttake(self, speed: float):
        self.outtake.set(speed)
        self.intake.set(speed)
