import commands2
import wpilib
import wpilib.drive
import constants
from networktables import NetworkTables
import rev


class ClimbingSubsystem(commands2.SubsystemBase):
    def __init__(self, liftArm, rotateArm, liftEncoder, rotateEncoder, liftArmUpLimitSwitch, rotateArmRobotLimitSwitch, liftArmDownLimitSwitch, rotateArmBackLimitSwitch) -> None:
        super().__init__()
        self.liftArm = liftArm
        self.rotateArm = rotateArm



        self.rotateEncoder = rotateEncoder
        self.liftEncoder = liftEncoder

        self.liftArmUpLimitSwitch = liftArmUpLimitSwitch
        self.liftArmDownLimitSwitch = liftArmDownLimitSwitch
        self.rotateArmBackLimitSwitch = rotateArmBackLimitSwitch
        self.rotateArmRobotLimitSwitch = rotateArmRobotLimitSwitch

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

    def getLiftArmLimitSwitchPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.liftArmUpLimitSwitch.get() == constants.liftArmUpLimitSwitchPressedValue or self.liftArmDownLimitSwitch.get() == constants.liftArmDownLimitSwitchPressedValue

    def getRotateArmLimitSwitchPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.rotateArmRobotLimitSwitch.get() == constants.rotateArmRobotLimitSwitchPressedValue or self.rotateArmBackLimitSwitch.get() == constants.rotateArmBackLimitSwitchPressedValue

    def setRotateArm(self, speed):
        self.rotateArm.set(speed)

    def setLiftArm(self, speed):
        self.liftArm.set(speed)
