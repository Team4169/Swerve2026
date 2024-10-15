# import commands2
# import wpilib
# import wpilib.drive
# import constants
# from constants import RobotConstants
# import ntcore
# import rev
# import math

# class ClimbingSubsystem(commands2.SubsystemBase):
#     def __init__(self) -> None:
#         super().__init__()
#         # commands2.SubsystemBase.__init__(self)
#         # ~ smartdashboard
#         self.sd = wpilib.SmartDashboard

#         #* Intake motors
#         self.leftClimbingMotor = rev.CANSparkMax(RobotConstants.climbingMotorLeftID, rev.CANSparkLowLevel.MotorType.kBrushless)
#         self.rightClimbingMotor = rev.CANSparkMax(RobotConstants.climbingMotorRightID, rev.CANSparkLowLevel.MotorType.kBrushless)


#     def runLeftClimbingMotor(self, speed: float):
#         self.leftClimbingMotor.set(speed)
    
#     def runRightClimbingMotor(self, speed: float):
#         self.rightClimbingMotor.set(speed)

#     def stopLeftClimbingMotor(self):
#         self.leftClimbingMotor.set(0)
    
#     def stopRightClimbingMotor(self):
#         self.rightClimbingMotor.set(0)
    
    