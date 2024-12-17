import commands2
import rev
import wpilib
import math
# import wpimath
# import phoenix5 #.configs.cancoder_configs.CANcoderConfiguration
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
#   from phoenix6.configs.cancoder_configs import CANcoderConfiguration
import phoenix6.hardware
from phoenix6.hardware import CANcoder
from commands2 import CommandScheduler


#from wpilib import DutyCycleEncoder
# import pandas 
# from pandas import Series

from constants import ModuleConstants, RobotConstants
class swervemodule(commands2.SubsystemBase):
    def __init__(self, drivingMotorID: int, turningMotorID:int, drivingMotorReversed: bool, turningMotorReversed: bool, 
                 absoluteEncoderId: int, absouteEncoderOffset: float, absoluteEncoderReversed: bool) -> None:
        super().__init__()

        #* Absolute Encoder
        #~Absolute Encoders help "remember" the location of the module.
        #~This is useful for when the robot is turned off and on again.
        self.absoluteEncoderOffsetRad = absouteEncoderOffset
        self.absoluteEncoderReversed = absoluteEncoderReversed
        #self.cancoderConfig = CANcoderConfiguration

        self.absoluteEncoder = CANcoder(absoluteEncoderId)
        #self.absoluteEncoder = DutyCycleEncoder(absoluteEncoderId)
        
        #* Swerve Module Motors init and Encoders
                # self.extendingArm = rev.CANSparkMax(RobotConstants.extendingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        
        self.drivingMotor = rev.CANSparkMax(drivingMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(turningMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)

        self.drivingMotor.setInverted(drivingMotorReversed)
        self.turningMotor.setInverted(turningMotorReversed)

        self.drivingEncoder = self.drivingMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()
        
        #~ this converts encoder ticks to meters, not entirely sure about how velocity works
        
        self.drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderRot2Meter)
        self.drivingEncoder.setVelocityConversionFactor(ModuleConstants.KDrivingEncoderRPM2MeterPerSec)

        self.turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        self.turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)

        #* PID Controllers
        self.turningPIDController = PIDController(ModuleConstants.kPTurning, 0, 0)
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)
    
        self.resetEncoders()

        #* smartdashboard init
        self.sd = wpilib.SmartDashboard
    #~ Helper functions

    def getDrivingPosition(self) -> float:
        return self.drivingEncoder.getPosition()
    
    def getTurningPostion(self) -> float:
        return self.turningEncoder.getPosition()
    
    def getDrivingVelocity(self) -> float:
        return self.drivingEncoder.getVelocity()
    
    def getTurningVelocity(self) -> float:
        return self.turningEncoder.getVelocity()
    
    def getAbsoluteEncoderRad(self) -> float:

        #? Change getAbsolutePosition to get_absolute_postion for cancoder
        # wpilib.SmartDashboard.putNumber("absEncoder", self.absoluteEncoder.get_absolute_position().getValue())
        wpilib.SmartDashboard.putNumber("absEncoder", self.absoluteEncoder.get_absolute_position().value_as_double)
        
        #wpilib.SmartDashboard.putData("absEncoder", self.absoluteEncoder.get_absolute_position())
       # print(self.absoluteEncoder.getAbsolutePosition())
        
        # angle = self.absoluteEncoder.getAbsolutePosition()  #? percent of full rotation
        # angle *= 2 * math.pi #? convert to radians
        # angle -= self.absoluteEncoderOffsetRad #? get acual location depending on the offset
        # return angle * (-1 if self.absoluteEncoderReversed else 1) #? reverse if needed
        
        #angle = self.absoluteEncoder.get_absolute_position().getValue() # ? percent of full rotation
        angle = self.absoluteEncoder.get_absolute_position().value_as_double
        angle *= 2 * math.pi #? convert to radians
        angle -= self.absoluteEncoderOffsetRad #? get acual location depending on the offset
        
        return angle * (-1 if self.absoluteEncoderReversed else 1) #?
        #reverse if needed
        
        
    def getSwerveModulePosition(self) -> SwerveModulePosition:
            return SwerveModulePosition(self.getDrivingPosition(), Rotation2d(self.getAbsoluteEncoderRad()))

    def resetEncoders(self):
        self.drivingEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())

    def getState(self) -> SwerveModuleState:
        #return SwerveModuleState(self.getDrivingVelocity(), Rotation2d(self.getTurningPostion()))
        return SwerveModuleState(self.drivingEncoder.getVelocity(), Rotation2d(self.getAbsoluteEncoderRad()))
    def setDesiredState(self, state:SwerveModuleState):
        #^ this prevents the wheels from resetting their position after every input is made
        #https://youtu.be/0Xi9yb1IMyA?t=577
        # if (abs(state.speed) < 0.001):
        #     self.stop()
        #     return
        self.state = SwerveModuleState.optimize(state, self.getState().angle) 
        self.drivingMotor.set(self.state.speed / RobotConstants.kphysicalMaxSpeedMetersPerSecond)
        
        #^ from my understanding of the above code, self.state.speed is apparently supposed to be in m/s.
        #^ as a result, dividing a set mps / max mps gets a value between 0 - 1 or -1 - 0 depending on if state.speed is negative
        #^ given that we don't know the max speed in m/s, I think we can do everything in terms of percentage ( -1, 1) right now,
        #^ untill we can accurately measure position and as a result, velocity, and acceleration
        #! make sure that driving position is accurate, then make sure driving velocity is accurate
        #! same for turning, get values of maxSpeedMetersPerSecond, maxAccel
        # self.drivingMotor.set(self.state.speed)
        # print(state.speed)

      
        #self.turningMotor.set(0) 
        self.turningPIDController.calculate(self.getAbsoluteEncoderRad(), self.state.angle.radians())

        self.sd.putNumber(f"Speed output", self.state.speed / RobotConstants.kphysicalMaxSpeedMetersPerSecond)        
        # self.sd.putString(f"Optimized state", str(self.state))
        # self.sd.putString(f"state", str(state))

        
# def setDesiredState(self, state: SwerveModuleState):
#     self.state = SwerveModuleState.optimize(state, self.getState().angle)

#     # Normalize speed to [-1, 1] range
#     driving_speed = self.state.speed / RobotConstants.kphysicalMaxSpeedMetersPerSecond
#     self.drivingMotor.set(driving_speed)

#     # PID control for turning
#     turning_output = self.turningPIDController.calculate(self.getAbsoluteEncoderRad(), self.state.angle.radians())
#     self.turningMotor.set(turning_output)

#     # Debugging
#     self.sd.putNumber("Driving Speed", driving_speed)
#     self.sd.putNumber("Turning Output", turning_output)

    def stop(self):
        self.drivingMotor.set(0)
        self.turningMotor.set(0)

    def setBrakeMode(self):
        self.drivingMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.turningMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

    def setCoastMode(self):
        self.drivingMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.turningMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)