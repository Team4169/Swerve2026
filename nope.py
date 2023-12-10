from wpilib.simulation import *
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
import pyfrc.physics    
import constants
import typing
import wpilib
from wpimath.system.plant import DCMotor
from constants import RobotConstants
if typing.TYPE_CHECKING:
    from robot import MyRobot
from wpimath.kinematics import SwerveDrive4Kinematics


class PhysicsEngine:
    """
    Simulates a 4-wheel mecanum robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        """
        
        self.robot = robot
        self.physics_controller = physics_controller
        
        # Motors
        self.lf_motor = SimDeviceSim("SPARK MAX", robot.swerve.frontLeft.drivingMotor.getDeviceId())
        self.lf_motor_pos = self.lf_motor.getDouble("Position")
        self.lf_turning_motor = SimDeviceSim("SPARK MAX",robot.swerve.frontLeft.turningMotor.getDeviceId())
        
        self.lr_motor = SimDeviceSim("SPARK MAX",robot.swerve.backLeft.drivingMotor.getDeviceId())
        self.lr_turning_motor = SimDeviceSim("SPARK MAX",robot.swerve.backLeft.turningMotor.getDeviceId())
        
        self.rf_motor = SimDeviceSim("SPARK MAX",robot.swerve.frontRight.drivingMotor.getDeviceId())
        self.rf_turning_motor = SimDeviceSim("SPARK MAX",robot.swerve.frontRight.turningMotor.getDeviceId())
        
        self.rr_motor = SimDeviceSim("SPARK MAX",robot.swerve.backRight.drivingMotor.getDeviceId())
        self.rr_turning_motor = SimDeviceSim("SPARK MAX",robot.swerve.backRight.turningMotor.getDeviceId())
        # Gyro
        self.gyro = AnalogGyroSim(wpilib.SerialPort.Port.kUSB1)

        # self.swerve_sim = drivetrains.four_motor_swerve_drivetrain(lr_motor = lr_motor, 
        #                                                   rf_motor = rf_motor, 
        #                                                   rr_motor = rr_motor,
        #                                                   lf_motor = lf_motor,
        #                                                   lr_angle = lr_angle,
        #                                                   rf_angle = rf_angle,
        #                                                   rr_angle = rr_angle,
        #                                                   lf_angle = lf_angle
        #                                                   )
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        
        # lf_motor = self.lf_motor.getInt("Velocity")
        # lf_angle = self.lf_turning_motor.getInt("Velocity")
        # lr_motor = self.lr_motor.getInt("Velocity")
        # lr_angle = self.lr_turning_motor.getInt("Velocity")
        # rf_motor = self.rf_motor.getInt("Velocity")
        # rf_angle = self.rf_turning_motor.getInt("Velocity")
        # rr_motor = self.rr_motor.getInt("Velocity")
        # rr_angle = self.rr_turning_motor.getInt("Velocity")
        print(self.lf_motor)
        
        # pose = self.physics_controller.drive(speeds, tm_diff)
        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        # self.gyro.setAngle(-pose.rotation().degrees())