import commands2
import wpilib
import wpilib.drive
import constants
import ntcore
import rev


class armsubsytem(commands2.SubsystemBase):
    def __init__(self, extendingArm, rotatingArm, grabbingArm, extendingArmLimitSwitchMin, extendingArmLimitSwitchMax, rotatingArmLimitSwitchMin, rotatingArmLimitSwitchMax, grabbingArmLimitSwitchMin, grabbingArmLimitSwitchMax, extendingArmEncoder, rotatingArmEncoder, grabbingArmEncoder) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        self.extendingArm = extendingArm
        self.rotatingArm = rotatingArm
        self.grabbingArm = grabbingArm

        # limit switches
        self.extendingArmLimitSwitchMin = extendingArmLimitSwitchMin
        self.extendingArmLimitSwitchMax = extendingArmLimitSwitchMax
        self.rotatingArmLimitSwitchMin = rotatingArmLimitSwitchMin
        self.rotatingArmLimitSwitchMax = rotatingArmLimitSwitchMax
        self.grabbingArmLimitSwitchMin = grabbingArmLimitSwitchMin
        self.grabbingArmLimitSwitchMax = grabbingArmLimitSwitchMax

        #encoders
        self.extendingArmEncoder = extendingArmEncoder
        self.rotatingArmEncoder = rotatingArmEncoder
        self.grabbingArmEncoder = grabbingArmEncoder



        # smartdashboard
        # # self.sd = NetworkTables.getTable("SmartDashboard")

    def resetEncoders(self) -> None:
        self.grabbingArmEncoder.reset()
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
        return self.grabbingArmEncoder.getCount()
    

    def getRotateArmEncoderDistance(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return self.rotateEncoder.getPosition()

    def getLiftArmLimitSwitchPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.liftArmUpLimitSwitch.get() == constants.liftArmUpLimitSwitchPressedValue or self.liftArmDownLimitSwitch.get() == constants.liftArmDownLimitSwitchPressedValue

    def getRotateArmLimitSwitchPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.rotateArmRobotLimitSwitch.get() == constants.rotateArmRobotLimitSwitchPressedValue or self.rotateArmBackLimitSwitch.get() == constants.rotateArmBackLimitSwitchPressedValue

    def setGrabbingArm(self, speed):
        self.grabbingArm.set(speed)

    def setCoast(self, isTrue):
        if not isTrue:
            self.rotateArm.setIdleMode(self.rotateArm.IdleMode.kBrake)
        else:
            self.rotateArm.setIdleMode(self.rotateArm.IdleMode.kCoast)

    def setLiftArm(self, speed):
        self.liftArm.set(speed)
