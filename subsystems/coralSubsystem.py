# import commands2
# import wpilib
# import wpilib.drive
# import constants
# from constants import RobotConstants
# import ntcore
# import rev
# import math
# import phoenix6



# class CoralSubsystem(commands2.SubsystemBase):
#     def __init__(self) -> None:
#         super().__init__()
#         # commands2.SubsystemBase.__init__(self)
#         # ~ smartdashboard
#         self.sd = wpilib.SmartDashboard

#         #* Intake motors
#         self.coralEntranceMotor = rev.SparkMax(RobotConstants.coralIntakeMotor1ID, rev.SparkLowLevel.MotorType.kBrushless)
#         self.coralLiftMotor = rev.SparkMax(RobotConstants.coralIntakeMotor2ID, rev.SparkLowLevel.MotorType.kBrushless)


         
#     def startCoral(self, speed: float):
#         self.coralEntranceMotor.set(speed)

#     def stopCoral(self):
#         self.coralEntranceMotor.set(0)

#     def liftCoral(self, speed: float):
#         self.coralLiftMotor.set(speed)

#     def stopLiftCoral(self):
#         self.coralLiftMotor.set(0)