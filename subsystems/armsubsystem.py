import commands2
import wpilib
import wpilib.drive
import constants
import ntcore
import rev
import math

# TODO step 5: move seat motor to 0 position with limit switch (reset)
# TODO     move down untill we hit the limit switch, then reset the encoder

class ArmSubsystem(commands2.SubsystemBase):
    def __init__(self, extendingArm, rotatingArm, grabbingArm, extendingArmLimitSwitchMin, extendingArmLimitSwitchMax, rotatingArmLimitSwitchMin, rotatingArmLimitSwitchMax, grabbingArmLimitSwitchOpen, grabbingArmLimitSwitchClosed, extendingArmEncoder, rotatingArmEncoder, grabbingArmEncoder) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = ntcore.NetworkTableInstance.getDefault().getTable("SmartDashboard")
        
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
        self.grabbingArmLimitSwitchClosed = grabbingArmLimitSwitchClosed

        #~ smartDashboard limit switches
        self.extendingArmLimitSwitchMinVal = self.sd.getDoubleTopic("ExtendArmLimitMin").publish()
        self.extendingArmLimitSwitchMaxVal = self.sd.getDoubleTopic("ExtendArmLimitMax").publish()
        self.rotatingArmLimitSwitchMinVal = self.sd.getDoubleTopic("RotArmLimitMin").publish()
        self.rotatingArmLimitSwitchMaxVal = self.sd.getDoubleTopic("RotArmLimitMax").publish()
        self.grabbingArmLimitSwitchOpenVal = self.sd.getDoubleTopic("GrabbingLimitOpen").publish()
        self.grabbingArmLimitSwitchClosedVal = self.sd.getDoubleTopic("GrabbingLimitClosed").publish()


        #* encoders
        self.extendingArmEncoder = extendingArmEncoder
        self.rotatingArmEncoder = rotatingArmEncoder
        self.grabbingArmEncoder = grabbingArmEncoder

        self.grabbingArmEncoderDegrees = 0.0
        self.previousGrabbingArmEncoderTicks = 0.0

        #~ smartDashboard encoders

        self.grabbingDegrees = self.sd.getDoubleTopic("grabbingDegrees").publish()
        self.extendingArmRevolutions = self.sd.getDoubleTopic("ExtendArmRevs").publish()
        self.rotatingArmRevolutions = self.sd.getDoubleTopic("RotArmRevs").publish()

        
        self.rotatingArmEncoderDegrees = self.rotatingArmEncoder.getPosition() * constants.rotatingArmRevPerArmDegree
        self.extendingArmEncoderPercent = self.extendingArmEncoder.getPosition() * constants.extendingArmRevPerArmPercent


        # self.left1.setSelectedSensorPosition(0, 0, 10)
        # self.right2.setSelectedSensorPosition(0, 0, 10)
        """Resets the drive encoders to currently read a position of 0."""
    
    #TODO: make functions return as degrees rather than ticks, tics/degree is required
# * Extending Arm functions
    def getExtendingArmLimitSwitchMinPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.extendingArmLimitSwitchMin.get()
    
    def getExtendingArmLimitSwitchMaxPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.extendingArmLimitSwitchMax.get()
    
    # limit Switch gaurds https://docs.google.com/spreadsheets/d/1Ywz5rC-dYjaaNjmlx7t1RDW8TRrBJ6oTPnMUaNfTfJ0/edit#gid=1791774740
    def setExtendingArmSpeed(self, speed):
        """Sets the speed of the extending arm"""
        if self.getExtendingArmLimitSwitchMaxPressed() and speed > 0:
            self.extendingArm.set(0)
        elif self.getExtendingArmLimitSwitchMinPressed() and speed < 0:
            self.extendingArm.set(0)
            self.resetExtendingArmEncoder()
        else:
            self.extendingArm.set(speed)
    
    #condition spreadsheet https://docs.google.com/spreadsheets/d/1Ywz5rC-dYjaaNjmlx7t1RDW8TRrBJ6oTPnMUaNfTfJ0/edit#gid=0
    def setExtendingArmSpeedWithAuto(self, speed):
        """Sets the speed of the extending arm"""
        #& if the rotating arm is between -7 and 24 degrees
         #& if the extending arm is greater than 75% of the way out
        if self.rotatingArmEncoderDegrees > constants.lowerArmAngleLimit and self.rotatingArmEncoderDegrees < 24 and self.extendingArmEncoderPercent > 75 and speed >= 0:
                #& move down to 75 % extension
                self.setExtendingArmPercent(70, .75) 
        else:
            self.setExtendingArmSpeed(speed)

    #~ TODO: apply the same logic to the other arm functions
    #~ TODO: ticks to angle and move untill angle is reached
    #~ TODO: make it so that it zeroes when it hits the limit switch
    # def setExtendingArmPercent(self, percent, speed):
    #     speed = abs(speed)
    #     """Sets the angle of the extending arm"""
    #     self.tolerance = 0
    #     if percent - self.tolerance > self.extendingArmEncoderPercent:
    #         self.setExtendingArmSpeed(speed)
    #     elif percent + self.tolerance < self.extendingArmEncoderPercent:
    #         self.setExtendingArmSpeed(-speed)
    #     else:
    #         self.setExtendingArmSpeed(0)
    
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

    def resetExtendingArmEncoder(self):
        """Resets the extending arm encoder"""
        self.extendingArmEncoder.setPosition(0)

    def zeroExtendingArm(self):
        """Zeroes the extending arm"""
        if self.getExtendingArmLimitSwitchMinPressed():
            self.resetExtendingArmEncoder()
            self.setExtendingArmSpeed(0)
        else: 
            self.setExtendingArmSpeed(-.1)
    
# * Rotating Arm functions
    def getRotatingArmLimitSwitchMinPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.rotatingArmLimitSwitchMin.get()
    
    def getRotatingArmLimitSwitchMaxPressed(self) -> bool:
        """Gets if the limit switch is pressed"""
        return self.rotatingArmLimitSwitchMax.get()
    
    def setRotatingArmSpeed(self, speed):
        """Sets the speed of the Rotating arm"""
        if self.getRotatingArmLimitSwitchMaxPressed() and speed > 0:
            self.rotatingArm.set(0)
        elif self.getRotatingArmLimitSwitchMinPressed() and speed < 0:
            self.rotatingArm.set(0)
            self.resetRotatingArmEncoder()
        else:
            self.rotatingArm.set(speed)

    def setRotatingArmSpeedWithAuto(self, speed):
        """Sets the speed of the Rotating arm"""
        if (self.rotatingArmEncoderDegrees > 68 and speed > 0 ):
            self.setRotatingArmAngle(65, .75)
        elif (self.rotatingArmEncoderDegrees <  constants.lowerArmAngleLimit and speed < 0):
            self.setRotatingArmAngle(-2, .75)
        else:
            self.setRotatingArmSpeed(speed)

    #~ TODO: make it so that it zeroes when it hits the limit switch
    def setRotatingArmAngle(self, angle, speed):
        speed = abs(speed)
        """Sets the angle of the Rotating arm"""
        self.tolerance = 0
        if angle - self.tolerance > self.rotatingArmEncoderDegrees:
            self.setRotatingArmSpeed(speed)
        elif angle + self.tolerance < self.rotatingArmEncoderDegrees:
            self.setRotatingArmSpeed(-speed)
        else:
            self.setRotatingArmSpeed(0)
    
    def resetRotatingArmEncoder(self):
        self.rotatingArmEncoder.setPosition(constants.lowerArmAngleLimit * constants.rotatingArmRevPerArmDegree) # ! we may not be able to set the encoder to negative degrees

    def zeroRotatingArm(self):
        """Zeroes the Rotating arm"""
        if self.getRotatingArmLimitSwitchMinPressed():
            self.resetRotatingArmEncoder()
            self.setRotatingArmSpeed(0)
        else: 
            self.setRotatingArmSpeed(-.1)

    
# * Grabbing Arm functions
    def getGrabbingArmLimitSwitchClosedPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.grabbingArmLimitSwitchClosed.get()
    
    def getGrabbingArmLimitSwitchOpenPressed(self) -> bool:
        """Gets if either limit switch is pressed"""
        return self.grabbingArmLimitSwitchOpen.get()
    
    def setGrabbingArmSpeedWithLimitSwitches(self, speed):
        """Sets the speed of the grabbing arm"""
        if self.getGrabbingArmLimitSwitchClosedPressed() and speed < 0:
            self.grabbingArm.set(0)
        elif self.getGrabbingArmLimitSwitchOpenPressed() and speed > 0:
            self.grabbingArm.set(0)
            self.resetGrabbingArmEncoder()
        else:
            self.grabbingArm.set(speed)
    
    def setGrabbingArmSpeed(self, speed):
        """for some reason the encoder ticks are significantly different when going down versus when going up"""
        # & Sets the speed
        self.setGrabbingArmSpeedWithLimitSwitches(speed)
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
    
    def grabCube(self):
        self.setGrabbingArmAngle(119, .75)
    
    def grabCone(self):
        self.setGrabbingArmAngle(88 , .75)

    #^ test this function

    def zeroGrabbingArm(self):
        if self.getGrabbingArmLimitSwitchOpenPressed():
            self.resetGrabbingArmEncoder()
            self.setGrabbingArmSpeed(0)
        else: 
            self.setGrabbingArmSpeed(-.1)
    
    def resetGrabbingArmEncoder(self) -> None:
        self.grabbingArmEncoder.reset()
        self.grabbingArmEncoderDegrees = 67.5
        self.previousGrabbingArmEncoderTicks = 0

    #* Object pickup functions
    def getTargetAngle(self, distance):
        """Gets the angle to the target"""
        return 180/math.pi * math.atan((distance + constants.cameraDistanceFromArm)/(constants.pivotDistanceFromGround-constants.armPickupHeight))-90
    


        