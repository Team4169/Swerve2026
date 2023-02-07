import commands2
import wpilib
import wpilib.drive
import ctre
import constants
import ntcore
import wpimath.controller
import navx
import rev
class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self, leftTalon, leftTalon2, rightTalon, rightTalon2) -> None:
        super().__init__()

        self.leftTalon = leftTalon
        self.leftTalon2 = leftTalon2
        self.rightTalon = rightTalon
        self.rightTalon2 = rightTalon2

        self.tpf = -900
        self.maxDriveSpeed = 0.6
        self.maxTurnSpeed = 0.6

        # smartdashboard
        self.sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")

        # Create PID Controller for Turning
        self.TurnkI = self.sd.getDoubleTopic("TurnkI").subscribe(0.0)
        self.TurnkP = self.sd.getDoubleTopic("TurnkP").subscribe(0.032)
        self.TurnkD = self.sd.getDoubleTopic("TurnkD").subscribe(0.0)
        #self.turnController = wpimath.controller.PIDController(self.TurnkP, self.TurnkI, self.TurnkD)
        #self.turnController.enableContinuousInput(-180.0, 180.0)
        #self.turnController.setTolerance(10.0)

        # Create PID Controller for Drive
        self.DrivekP = self.sd.getDoubleTopic("DrivekP").subscribe(0.02)
        self.DrivekI = self.sd.getDoubleTopic("DrivekI").subscribe(0.02)
        self.DrivekD = self.sd.getDoubleTopic("DrivekD").subscribe(0.0005)
        #self.driveController = wpimath.controller.PIDController(self.DrivekP, self.DrivekI, self.DrivekD)
        #self.driveController.setTolerance(-0.1 * self.tpf)

        # gyro
        self.gyro = navx.AHRS(wpilib.SerialPort.Port.kUSB1)
        self.gyroOut = self.sd.getDoubleTopic("Gyro Yaw").publish()
        self.gyroOut.set(self.gyro.getYaw())
        
        # The robot's drive
        self.rightTalon2.setInverted(True)
        self.rightTalon.setInverted(True)
        # self.drive = wpilib.drive.DifferentialDrive(
        #     wpilib.MotorControllerGroup(self.leftTalon, self.leftTalon2),
        #     wpilib.MotorControllerGroup(self.rightTalon, self.rightTalon2)
        #     )

        # The left-side drive encoder
        # NOTE FROM NOAH - I commented the encoders out, will use the talon interface to get encoders
        # self.leftEncoder = wpilib.Encoder(
        #     *constants.kLeftEncoderPorts,
        #     reverseDirection=constants.kLeftEncoderReversed
        # )
        #
        # # The right-side drive encoder
        #self.rightEncoder = wpilib.Encoder(
        #     *constants.kRightEncoderPorts,
        #     reverseDirection=constants.kRightEncoderReversed
        # )
        # self.leftTalon.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        # self.rightTalon.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)

        # Sets the distance per pulse for the encoders
        # NOTE FROM NOAH - Expirement with these two following lines later, for now commenting them out
        # self.leftEncoder.setDistancePerPulse(constants.kEncoderDistancePerPulse)
        # self.rightEncoder.setDistancePerPulse(constants.kEncoderDistancePerPulse)

    # def arcadeDrive(self, fwd: float, rot: float) -> None:
    #     """
    #     Drives the robot using arcade controls.

    #     :param fwd: the commanded forward movement
    #     :param rot: the commanded rotation
    #     """
    #     self.drive.arcadeDrive(fwd, rot)

    # def resetEncoders(self) -> None:
    #     self.leftTalon.setSelectedSensorPosition(0, 0, 10)
    #     self.rightTalon.setSelectedSensorPosition(0, 0, 10)
    #     """Resets the drive encoders to currently read a position of 0."""

    # def getAverageEncoderDistance(self) -> float:
    #     """Gets the average distance of the TWO encoders."""
    #     # self.sd.putValue("Left Encoder Value", self.leftTalon.getSelectedSensorPosition())
    #     # self.sd.putValue("Right Encoder Value", self.rightTalon.getSelectedSensorPosition())
    #     return (self.leftTalon.getSelectedSensorPosition()  * 12 / self.tpf)

    # def getAverageEncoderTicks(self) -> float:
    #     """Gets the average distance of the TWO encoders."""
    #     # self.sd.putValue("Left Encoder Value", self.leftTalon.getSelectedSensorPosition())
    #     # self.sd.putValue("Right Encoder Value", self.rightTalon.getSelectedSensorPosition())
    #     return self.leftTalon.getSelectedSensorPosition() * -1

    # def setMaxOutput(self, maxOutput: float):
    #     """
    #     Sets the max output of the drive. Useful for scaling the
    #     drive to drive more slowly.
    #     """
    #     self.drive.setMaxOutput(maxOutput)

    # def validateDriveSpeed(self, speed):
    #     if speed > self.maxDriveSpeed:
    #         return self.maxDriveSpeed
    #     if speed < -1 * self.maxDriveSpeed:
    #         return -1 * self.maxDriveSpeed
    #     return speed

    # def validateTurnSpeed(self, turnSpeed):
    #     if turnSpeed > self.maxTurnSpeed:
    #         return self.maxTurnSpeed
    #     if turnSpeed < -1 * self.maxTurnSpeed:
    #         return -1 * self.maxTurnSpeed
    #     return turnSpeed
