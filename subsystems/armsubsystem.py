import commands2
import wpilib
import wpilib.drive
import constants
import ntcore
import rev


class ArmSubsystem(commands2.SubsystemBase):
    def __init__(self, extendingArm, rotatingArm, grabbingArm, extendingArmLimitSwitchMin, extendingArmLimitSwitchMax, rotatingArmLimitSwitchMin, rotatingArmLimitSwitchMax, grabbingArmLimitSwitchOpen, grabbingArmLimitSwitchClosed, extendingArmEncoder, rotatingArmEncoder, grabbingArmEncoder) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        self.sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
        
        self.grabbingTicks = self.sd.getDoubleTopic("grabbingDegrees").publish()
        
        self.realTicks = self.sd.getDoubleTopic("realTicks").publish()
        self.previousTicks = self.sd.getDoubleTopic("previousTicks").publish()
        self.grabbingLimitSwitchOpenVal = self.sd.getDoubleTopic("GrabbingLimitOpen").publish()

        self.extendingArm = extendingArm
        self.rotatingArm = rotatingArm
        self.grabbingArm = grabbingArm

        # limit switches
        self.extendingArmLimitSwitchMin = extendingArmLimitSwitchMin
        self.extendingArmLimitSwitchMax = extendingArmLimitSwitchMax
        self.rotatingArmLimitSwitchMin = rotatingArmLimitSwitchMin
        self.rotatingArmLimitSwitchMax = rotatingArmLimitSwitchMax
        self.grabbingArmLimitSwitchOpen = grabbingArmLimitSwitchOpen
        self.grabbingArmLimitSwitchMax = grabbingArmLimitSwitchClosed

        #encoders
        self.extendingArmEncoder = extendingArmEncoder
        self.rotatingArmEncoder = rotatingArmEncoder
        self.grabbingArmEncoder = grabbingArmEncoder

        self.grabbingArmEncoderDegrees = 0
        self.previousGrabbingArmEncoderTicks = 0


        # smartdashboard
        # # self.sd = NetworkTables.getTable("SmartDashboard")

    def resetEncoders(self) -> None:
        self.grabbingArmEncoder.reset()
        self.grabbingArmEncoderDegrees = 0
        self.previousGrabbingArmEncoderTicks = 0

        # self.left1.setSelectedSensorPosition(0, 0, 10)
        # self.right2.setSelectedSensorPosition(0, 0, 10)
        """Resets the drive encoders to currently read a position of 0."""

    #TODO: make functions return as degrees rather than ticks, tics/degree is required
    def getRotatingArmEncoderAngle(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return self.rotatingArmEncoder.getPosition()
    
    def getExtendingArmEncoderDistance(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return self.extendingArmEncoder.getPosition()
    
    def getGrabbingArmEncoderDistance(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return self.grabbingArmEncoderDegrees
    

    def getRotateArmEncoderDistance(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return self.rotateEncoder.getPosition()

    def getLiftArmLimitSwitchPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.liftArmUpLimitSwitch.get() == constants.liftArmUpLimitSwitchPressedValue or self.liftArmDownLimitSwitch.get() == constants.liftArmDownLimitSwitchPressedValue

    def getGrabbingArmLimitSwitchOpenPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.grabbingArmLimitSwitchOpen.get() == constants.grabbingArmOpenLimitSwitchPressedValue
    
    def setGrabbingArmSpeed(self, speed):
        
        self.grabbingArm.set(speed)
        self.current = self.grabbingArmEncoder.get()

        self.realTicks.set(self.grabbingArmEncoder.get())
        self.previousTicks.set(self.previousGrabbingArmEncoderTicks)
        self.diff = self.current - self.previousGrabbingArmEncoderTicks
        if speed < 0:
            self.grabbingArmEncoderDegrees -= self.diff / constants.negativeTicksPerDeg
        elif speed > 0:
            self.grabbingArmEncoderDegrees += self.diff / constants.positiveTicksPerDeg
        self.previousGrabbingArmEncoderTicks = self.current

    def setGrabbingArmAngle(self, angle, speed):
        self.tolerance = .5
        if angle - self.tolerance > self.grabbingArmEncoderDegrees:
            self.setGrabbingArmSpeed(speed)
        elif angle + self.tolerance < self.grabbingArmEncoderDegrees:
            self.setGrabbingArmSpeed(-speed)
        else:
            self.setGrabbingArmSpeed(0)


    def setCoast(self, isTrue):
        if not isTrue:
            self.rotateArm.setIdleMode(self.rotateArm.IdleMode.kBrake)
        else:
            self.rotateArm.setIdleMode(self.rotateArm.IdleMode.kCoast)

    def setLiftArm(self, speed):
        self.liftArm.set(speed)
