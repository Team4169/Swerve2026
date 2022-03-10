from subsystems.drivesubsystem import DriveSubsystem
import commands2
import wpilib
#time in seconds
ticksDistance = 0
class climbUpDown(commands2.CommandBase): 
    def __init__(self, power: float) -> None:
        super().__init__()
        self.power = power
    
    def initialize(self):
        pass
    
    def execute(self) -> None:
        self.liftArm.set(self.power)
        
    def end(self):
        self.liftArm.set(0)
    
    def isFinished(self) -> bool:
        return self.liftArmEncoder.getPosition() > ticksDistance or self.liftArmEncoder.getPosition() < -ticksDistance 