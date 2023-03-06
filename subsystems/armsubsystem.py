import commands2
import wpilib
import wpilib.drive
import constants
import ntcore
import rev

# TODO step 5: move seat motor to 0 position with limit switch (reset)
# TODO     move down untill we hit the limit switch, then reset the encoder

class ArmSubsystem(commands2.SubsystemBase):
    def __init__(self, extendingArm, rotatingArm, grabbingArm, extendingArmLimitSwitchMin, extendingArmLimitSwitchMax, rotatingArmLimitSwitchMin, rotatingArmLimitSwitchMax, grabbingArmLimitSwitchOpen, grabbingArmLimitSwitchClosed, extendingArmEncoder, rotatingArmEncoder, grabbingArmEncoder) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # * smartdashboard
        self.sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
        
        self.grabbingDegrees = self.sd.getDoubleTopic("grabbingDegrees").publish()
        self.extendingArmRevolutions = self.sd.getDoubleTopic("ExtendArmRevs").publish()
        self.rotatingArmRevolutions = self.sd.getDoubleTopic("RotArmRevs").publish()
        self.grabbingLimitSwitchOpenVal = self.sd.getDoubleTopic("GrabbingLimitOpen").publish()

        #* Arm motors
        self.extendingArm = extendingArm
        self.rotatingArm = rotatingArm
        self.grabbingArm = grabbingArm

        #* limit switches
        self.extendingArmLimitSwitchMin = extendingArmLimitSwitchMin
        self.extendingArmLimitSwitchMax = extendingArmLimitSwitchMax
        self.rotatingArmLimitSwitchMin = rotatingArmLimitSwitchMin
        self.rotatingArmLimitSwitchMax = rotatingArmLimitSwitchMax
        self.grabbingArmLimitSwitchOpen = grabbingArmLimitSwitchOpen
        self.grabbingArmLimitSwitchMax = grabbingArmLimitSwitchClosed

        #* encoders
        self.extendingArmEncoder = extendingArmEncoder
        self.rotatingArmEncoder = rotatingArmEncoder
        self.grabbingArmEncoder = grabbingArmEncoder

        self.grabbingArmEncoderDegrees = 0
        self.previousGrabbingArmEncoderTicks = 0

        self.rotatingArmEncoderDegrees = self.rotatingArmEncoder.getPosition() / 360
        self.extendingArmEncoderPercent = self.extendingArmEncoder.getPosition() * constants.extendingArmRevPerArmPercent

    def resetEncoders(self) -> None:
        self.grabbingArmEncoder.reset()
        self.grabbingArmEncoderDegrees = 0
        self.previousGrabbingArmEncoderTicks = 0

        # self.left1.setSelectedSensorPosition(0, 0, 10)
        # self.right2.setSelectedSensorPosition(0, 0, 10)
        """Resets the drive encoders to currently read a position of 0."""
    
    #TODO: make functions return as degrees rather than ticks, tics/degree is required
# * Extending Arm functions
    def getExtendingArmLimitSwitchMinPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.extendingArmLimitSwitchMin.get() == constants.extendingArmMinLimitSwitchPressedValue
    
    def getExtendingArmLimitSwitchMaxPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.extendingArmLimitSwitchMax.get() == constants.extendingArmMaxLimitSwitchPressedValue
    
    # limit Switch gaurds https://docs.google.com/spreadsheets/d/1Ywz5rC-dYjaaNjmlx7t1RDW8TRrBJ6oTPnMUaNfTfJ0/edit#gid=1791774740
    def setExtendingArmSpeed(self, speed):
        """Sets the speed of the extending arm"""
        if self.getExtendingArmLimitSwitchMaxPressed() and speed > 0:
            self.extendingArm.set(0)
        elif self.getExtendingArmLimitSwitchMinPressed() and speed < 0:
            self.extendingArm.set(0)
        else:
            self.extendingArm.set(speed)
    
    #condition spreadsheet https://docs.google.com/spreadsheets/d/1Ywz5rC-dYjaaNjmlx7t1RDW8TRrBJ6oTPnMUaNfTfJ0/edit#gid=0
    def setExtendingArmSpeedWithAuto(self, speed):
        """Sets the speed of the extending arm"""
        #& if the rotating arm is between -7 and 24 degrees
         #& if the extending arm is greater than 75% of the way out
        if self.rotatingArmEncoderDegrees > -7 and self.rotatingArmEncoderDegrees < 24 and self.extendingArmEncoderPercent > 75 and speed >= 0:
                #& move down to 75 % extension
                self.setExtendingArmPercent(70, .75) 
        else:
            self.setExtendingArmSpeed(speed)

    #~ TODO: apply the same logic to the other arm functions
    #~ TODO: ticks to angle and move untill angle is reached
    #~ TODO: make it so that it zeroes when it hits the limit switch
    def setExtendingArmPercent(self, percent, speed):
        speed = abs(speed)
        """Sets the angle of the extending arm"""
        self.tolerance = 0
        if percent - self.tolerance > self.extendingArmEncoderPercent:
            self.setExtendingArmSpeed(speed)
        elif percent + self.tolerance < self.extendingArmEncoderPercent:
            self.setExtendingArmSpeed(-speed)
        else:
            self.setExtendingArmSpeed(0)
    
    #^ test this code, it will automatically apply limits on the extending arm
    def setExtendingArmPercentWithAuto(self, percent, speed):
        speed = abs(speed)
        """Sets the angle of the extending arm"""
        if percent - self.tolerance > self.extendingArmEncoderPercent:
            self.setGrabbingArmSpeedWithAuto(speed)
        elif percent + self.tolerance < self.extendingArmEncoderPercent:
            self.setGrabbingArmSpeedWithAuto(-speed)
        else:
            self.setGrabbingArmSpeedWithAuto(0)

    def zeroExtendingArm(self):
        """Zeroes the extending arm"""
        pass
    
# * Rotating Arm functions
    def getRotatingArmLimitSwitchMinPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.RotatingArmLimitSwitchMin.get() == constants.RotatingArmMinLimitSwitchPressedValue
    
    def getRotatingArmLimitSwitchMaxPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.RotatingArmLimitSwitchMax.get() == constants.RotatingArmMaxLimitSwitchPressedValue
    
    def setRotatingArmSpeed(self, speed):
        """Sets the speed of the Rotating arm"""
        self.rotatingArm.set(speed)

    def setRotatingArmSpeedWithAuto(self, speed):
        """Sets the speed of the Rotating arm"""
        if (self.rotatingArmEncoderDegrees > 68 and speed > 0 ):
            self.setRotatingArmAngle(65, .75)
        elif (self.rotatingArmEncoderDegrees <  -7 and speed < 0):
            self.setRotatingArmAngle(-2, .75)
        else:
            self.rotatingArm.set(speed)

    #~ TODO: make it so that it zeroes when it hits the limit switch
    def setRotatingArmAngle(self, angle, speed):
        speed = abs(speed)
        """Sets the angle of the Rotating arm"""
        self.tolerance = 0
        if angle - self.tolerance > self.grabbingArmEncoderDegrees:
            self.rotatingArm.set(speed)
        elif angle + self.tolerance < self.grabbingArmEncoderDegrees:
            self.rotatingArm.set(-speed)
        else:
            self.rotatingArm.set(0)
        
    def zeroRotatingArm(self):
        """Zeroes the Rotating arm"""
        pass
    
# * Grabbing Arm functions
    def getGrabbingArmLimitSwitchClosedPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.grabbingArmLimitSwitchOpen.get() == constants.grabbingArmOpenLimitSwitchPressedValue
    
    def getGrabbingArmLimitSwitchOpenPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.grabbingArmLimitSwitchOpen.get() == constants.grabbingArmOpenLimitSwitchPressedValue
    
    def setGrabbingArmSpeed(self, speed):
        """for some reason the encoder ticks are significantly different when going down versus when going up"""
        # & Sets the speed
        self.grabbingArm.set(speed)
        #& Updates the current encoder location
        self.current = self.grabbingArmEncoder.get()
        #& Updates the smartdashboard
        # //self.realTicks.set(self.grabbingArmEncoder.get())
        # //self.previousTicks.set(self.previousGrabbingArmEncoderTicks)
        #& gets the difference in ticks from the previous location
        self.diff = self.current - self.previousGrabbingArmEncoderTicks
        #& converts the diff into degrees depending on direction of travel
        if speed < 0:
            self.grabbingArmEncoderDegrees -= self.diff / constants.negativeTicksPerDeg
        elif speed > 0:
            self.grabbingArmEncoderDegrees += self.diff / constants.positiveTicksPerDeg
        #& changes current
        self.previousGrabbingArmEncoderTicks = self.current

    def setGrabbingArmAngle(self, angle, speed):
        self.tolerance = .5 #? should this be in the constants file?
        if angle - self.tolerance > self.grabbingArmEncoderDegrees:
            self.setGrabbingArmSpeed(speed)
        elif angle + self.tolerance < self.grabbingArmEncoderDegrees:
            self.setGrabbingArmSpeed(-speed)
        else:
            self.setGrabbingArmSpeed(0)
    
    #^ test this function
    def zeroGrabbingArm(self):
        if self.getGrabbingArmLimitSwitchOpenPressed():
            self.resetEncoders()
            self.setGrabbingArmSpeed(0)
        else: 
            self.setGrabbingArmSpeed(-.1)