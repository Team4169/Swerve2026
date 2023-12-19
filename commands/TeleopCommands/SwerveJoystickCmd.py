import commands2, constants, wpilib, navx, threading, time, math
from constants import OIConstants, RobotConstants
from wpimath.filter import SlewRateLimiter
from commands2 import CommandBase 
from wpilib import PS4Controller
from wpilib import XboxController

from subsystems.swervesubsystem import SwerveSubsystem
from wpimath.kinematics import ChassisSpeeds

class SwerveJoystickCmd(CommandBase):

    def __init__(self, swerve: SwerveSubsystem, driverController:XboxController):
        super().__init__()
        self.swerve = swerve
        self.driverController = driverController
        self.addRequirements(self.swerve)

        # create Slew limiter
        self.xLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAccelerationMetersPerSecSquared)
        self.yLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAccelerationMetersPerSecSquared)
        self.zRotLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAngularAccelerationRadiansPerSecSquared)



    def initialize(self):
        pass
    
    def execute(self):
        self.xSpeed = -self.driverController.getLeftX() * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ySpeed = self.driverController.getLeftY() * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.zRotation = self.driverController.getRightX()

        # 1. Get the joystick values and apply deadzone
        
        # print(self.ySpeed)
        # print(self.zRotation)
        
        self.xSpeed = self.xSpeed if (abs(self.xSpeed) > OIConstants.deadzone) else 0
        self.ySpeed = self.ySpeed if (abs(self.ySpeed) > OIConstants.deadzone) else 0
        self.zRotation = self.zRotation if (abs(self.zRotation) > OIConstants.deadzone) else 0
        
        self.swerve.sd.putNumber("xSpeed", self.xSpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.swerve.sd.putNumber("ySpeed", self.ySpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.swerve.sd.putNumber("Zspeed", self.zRotation)
        # # 2. Add rateLimiter to smooth the joystick values
        # self.xSpeed = self.xLimiter.calculate(self.xSpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        # self.ySpeed = self.yLimiter.calculate(self.ySpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        # self.zRotation = self.zRotLimiter.calculate(self.zRotation) \
        #             * RobotConstants.kTeleopDriveMaxAngularSpeedRadiansPerSecond
        
        
        

        # if self.feildOriented:
        #     chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, 
        #                                                          self.zRotation, self.swerve.getRotation2d())
        # else:
        chasisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.zRotation)

        # 3. convert chasis speeds to module states
        moduleStates = RobotConstants.kDriveKinematics.toSwerveModuleStates(chasisSpeeds)

        self.swerve.setModuleStates(moduleStates)
        

    def end(self, interrupted):
        self.swerve.stopModules()

    def isFinished(self):
        return False