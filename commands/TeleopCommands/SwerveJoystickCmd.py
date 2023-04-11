import commands2, constants, wpilib, navx, threading, time, math
from wpimath.filter import SlewRateLimiter
from commands2 import CommandBase 

from subsystems.swervesubsystem import SwerveSubsystem
from wpimath.kinematics import ChassisSpeeds

class SwerveJoystickCmd(CommandBase):

    def driveSwerve(self, swerve: SwerveSubsystem, xSpeed:float, 
                    YSpeed:float, ZRotation:float, feildOriented:bool):
        self.swerve = swerve
        self.xSpeed = xSpeed
        self.YSpeed = YSpeed
        self.ZRotation = ZRotation
        self.feildOriented = feildOriented
        self.addRequirements(swerve)

        # create Slew limiter
        self.xLimiter = SlewRateLimiter(constants.kTeleopDriveMaxAccelerationUnitsPerSec)
        self.yLimiter = SlewRateLimiter(constants.kTeleopDriveMaxAccelerationUnitsPerSec)
        self.zRotLimiter = SlewRateLimiter(constants.KTeleopDriveMaxAngularAccelerationUnitsPerSec)



    def initialize(self):
        pass

    def execute(self):
        # 1. Get the joystick values and apply deadzone
        self.xSpeed = self.xSpeed if (abs(self.xSpeed) > constants.deadzone) else 0
        self.YSpeed = self.YSpeed if (abs(self.YSpeed) > constants.deadzone) else 0
        self.ZRotation = self.ZRotation if (abs(self.ZRotation) > constants.deadzone) else 0
        
        # 2. Add rateLimiter to smooth the joystick values
        self.xSpeed = self.xLimiter.calculate(self.xSpeed) * constants.kTeleopDriveMaxSpeedMetersPerSecond
        self.YSpeed = self.yLimiter.calculate(self.YSpeed) * constants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ZRotation = self.zRotLimiter.calculate(self.ZRotation) \
                    * constants.KTeleopDriveMaxAngularSpeedRadiansPerSecond
        if self.feildOriented:
            chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.YSpeed, 
                                                                 self.ZRotation, self.swerve.getRotation2d())
        else:
            chasisSpeeds = ChassisSpeeds(self.xSpeed, self.YSpeed, self.ZRotation)

        # 3. convert chasis speeds to module states
        moduleStates = constants.kDriveKinematics.toSwerveModuleStates(chasisSpeeds)

        self.swerve.setModuleStates(moduleStates)
        

    def end(self, interrupted):
        self.swerve.stopModuels()

    def isFinished(self):
        return False