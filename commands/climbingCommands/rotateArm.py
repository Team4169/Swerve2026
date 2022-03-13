from subsystems.drivesubsystem import DriveSubsystem
import commands2
import wpilib
#time in seconds
ticksDistance = 0
class rotateArm(commands2.CommandBase): 
    def __init__(self, power: float, distance: float) -> None:
        super().__init__()
        self.power = power
        self.distance = distance
    
    def initialize(self):
        pass
    
    def execute(self) -> None:
        self.rotateArm.set(self.power)
        
    def end(self):
        self.rotateArm.set(0)
    
    def isFinished(self) -> bool:
        return self.rotateArmEncoder.getPosition() >  self.distance or self.rotateArmEncoder.getPosition() <  -self.distance