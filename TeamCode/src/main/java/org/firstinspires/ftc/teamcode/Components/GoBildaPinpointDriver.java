package org.firstinspires.ftc.teamcode.Components;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;


import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.teamcode.Wrapper.Pose2d;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;


@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer",
        xmlTag = "goBILDAPinpoint",
        description ="goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)

public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    private int deviceStatus   = 0;
    private int loopTime       = 0;
    private int xEncoderValue  = 0;
    private int yEncoderValue  = 0;
    private float xPosition    = 0;
    private float yPosition    = 0;
    private float hOrientation = 0;
    private float xVelocity    = 0;
    private float yVelocity    = 0;
    private float hVelocity    = 0;

    private static final float ticksPerInch = 505.316944317f;
    private static final boolean xEncoderNormal = true;
    private static final boolean yEncoderNormal = true;
    private static final float xEncoderOffset = 3.8897272f;
    private static final float yEncoderOffset = -5.4527559f;

    public static final byte DEFAULT_ADDRESS = 0x31;
    public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }

    private enum Register {
        DEVICE_ID       (1),
        DEVICE_VERSION  (2),
        DEVICE_STATUS   (3),
        DEVICE_CONTROL  (4),
        LOOP_TIME       (5),
        X_ENCODER_VALUE (6),
        Y_ENCODER_VALUE (7),
        X_POSITION      (8),
        Y_POSITION      (9),
        H_ORIENTATION   (10),
        X_VELOCITY      (11),
        Y_VELOCITY      (12),
        H_VELOCITY      (13),
        TICK_PER_IN     (14),
        X_POD_OFFSET    (15),
        Y_POD_OFFSET    (16),
        YAW_SCALAR      (17),
        BULK_READ       (18);

        private final int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }
    }

    public enum DeviceStatus{
        NOT_READY                (0),
        READY                    (1),
        CALIBRATING              (1 << 1),
        FAULT_X_POD_NOT_DETECTED (1 << 2),
        FAULT_Y_POD_NOT_DETECTED (1 << 3),
        FAULT_NO_PODS_DETECTED   (1 << 2 | 1 << 3),
        FAULT_IMU_RUNAWAY        (1 << 4),
        FAULT_BAD_READ           (1 << 5);

        private final int status;

        DeviceStatus(int status){
            this.status = status;
        }
    }

    public void init(){
        //encoder resolution
        writeByteArray(Register.TICK_PER_IN,(floatToByteArray((float) ticksPerInch,ByteOrder.LITTLE_ENDIAN)));

        //encoder direction
        if (xEncoderNormal){
            writeInt(Register.DEVICE_CONTROL,1<<5);
        }else {
            writeInt(Register.DEVICE_CONTROL,1<<4);
        }

        if (yEncoderNormal){
            writeInt(Register.DEVICE_CONTROL,1<<3);
        }else {
            writeInt(Register.DEVICE_CONTROL,1<<2);
        }

        //encoder offsets
        writeFloat(Register.X_POD_OFFSET, (float) xEncoderOffset);
        writeFloat(Register.Y_POD_OFFSET, (float) yEncoderOffset);
    }

    private void writeInt(final Register reg, int i){
        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i,ByteOrder.LITTLE_ENDIAN));
    }

    private int readInt(Register reg){
        return byteArrayToInt(deviceClient.read(reg.bVal,4), ByteOrder.LITTLE_ENDIAN);
    }

    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder){
        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
    }

    private float readFloat(Register reg){
        return byteArrayToFloat(deviceClient.read(reg.bVal,4),ByteOrder.LITTLE_ENDIAN);
    }

    private byte [] floatToByteArray (float value, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
    }

    private void writeByteArray (Register reg, byte[] bytes){
        deviceClient.write(reg.bVal,bytes);
    }

    private void writeFloat (Register reg, float f){
        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
        deviceClient.write(reg.bVal,bytes);
    }

    private DeviceStatus lookupStatus (int s){
        if ((s & DeviceStatus.CALIBRATING.status) != 0){
            return DeviceStatus.CALIBRATING;
        }
        boolean xPodDetected = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
        boolean yPodDetected = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;

        if(!xPodDetected  && !yPodDetected){
            return DeviceStatus.FAULT_NO_PODS_DETECTED;
        }
        if (!xPodDetected){
            return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        }
        if (!yPodDetected){
            return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        }
        if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0){
            return DeviceStatus.FAULT_IMU_RUNAWAY;
        }
        if ((s & DeviceStatus.READY.status) != 0){
            return DeviceStatus.READY;
        }
        if ((s & DeviceStatus.FAULT_BAD_READ.status) != 0){
            return DeviceStatus.FAULT_BAD_READ;
        }
        else {
            return DeviceStatus.NOT_READY;
        }
    }

    private float isPositionCorrupt(float oldValue, float newValue, float threshold, boolean bulkUpdate) {
        boolean noData   = bulkUpdate && (loopTime < 1);
        boolean isCorrupt = noData
                || Float.isNaN(newValue)
                || Math.abs(newValue - oldValue) > threshold;

        if (!isCorrupt) return newValue;

        deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
        return oldValue;
    }

    private float isVelocityCorrupt(float oldValue, float newValue, float threshold) {
        boolean noData   = (loopTime <= 1);
        boolean isCorrupt = noData
                || Float.isNaN(newValue)
                || Math.abs(newValue) > threshold;

        if (!isCorrupt) return newValue;

        deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
        return oldValue;
    }

    public void update(){
        final int positionThreshold = 250; //more than one FTC field in in
        final int headingThreshold = 10; //About 20 full rotations in Radians
        final int velocityThreshold = 200; // 200 in/sec is faster than an FTC robot should be going...
        final int headingVelocityThreshold = 120; //About 20 rotations per second

        float oldPosX = xPosition;
        float oldPosY = yPosition;
        float oldPosH = hOrientation;
        float oldVelX = xVelocity;
        float oldVelY = yVelocity;
        float oldVelH = hVelocity;

        byte[] bArr   = deviceClient.read(Register.BULK_READ.bVal, 40);
        ByteBuffer buf = ByteBuffer.wrap(bArr).order(ByteOrder.LITTLE_ENDIAN);

        deviceStatus  = buf.getInt();
        loopTime      = buf.getInt();
        xEncoderValue = buf.getInt();
        yEncoderValue = buf.getInt();
        xPosition     = buf.getFloat();
        yPosition     = buf.getFloat();
        hOrientation  = buf.getFloat();
        xVelocity     = buf.getFloat();
        yVelocity     = buf.getFloat();
        hVelocity     = buf.getFloat();

        /*
         * Check to see if any of the floats we have received from the device are NaN or are too large
         * if they are, we return the previously read value and alert the user via the DeviceStatus Enum.
         */
        xPosition    = isPositionCorrupt(oldPosX, xPosition, positionThreshold, true);
        yPosition    = isPositionCorrupt(oldPosY, yPosition, positionThreshold, true);
        hOrientation = isPositionCorrupt(oldPosH, hOrientation, headingThreshold, true);
        xVelocity    = isVelocityCorrupt(oldVelX, xVelocity, velocityThreshold);
        yVelocity    = isVelocityCorrupt(oldVelY, yVelocity, velocityThreshold);
        hVelocity    = isVelocityCorrupt(oldVelH, hVelocity, headingVelocityThreshold);

    }

    public void recalibrateIMU(){writeInt(Register.DEVICE_CONTROL,1<<0);}

    public void resetPosAndIMU(){writeInt(Register.DEVICE_CONTROL,1<<1);}

    public void setYawScalar(double yawOffset){
        writeByteArray(Register.YAW_SCALAR,(floatToByteArray((float) yawOffset, ByteOrder.LITTLE_ENDIAN)));
    }

    public Pose2d setPosition(Pose2d pos){
        writeByteArray(Register.X_POSITION,(floatToByteArray((float) pos.x(), ByteOrder.LITTLE_ENDIAN)));
        writeByteArray(Register.Y_POSITION,(floatToByteArray((float) pos.y(),ByteOrder.LITTLE_ENDIAN)));
        writeByteArray(Register.H_ORIENTATION,(floatToByteArray((float) pos.heading(),ByteOrder.LITTLE_ENDIAN)));
        return pos;
    }
    public void setPosX(double posX){
        writeByteArray(Register.X_POSITION,(floatToByteArray((float) posX, ByteOrder.LITTLE_ENDIAN)));
    }

    public void setPosY(double posY){
        writeByteArray(Register.Y_POSITION,(floatToByteArray((float) posY, ByteOrder.LITTLE_ENDIAN)));
    }

    public void setHeading(double heading){
        writeByteArray(Register.H_ORIENTATION,(floatToByteArray((float) heading, ByteOrder.LITTLE_ENDIAN)));
    }

    public int getDeviceID(){return readInt(Register.DEVICE_ID);}

    public int getDeviceVersion(){return readInt(Register.DEVICE_VERSION); }

    public float getYawScalar(){return readFloat(Register.YAW_SCALAR); }

    public DeviceStatus getDeviceStatus(){return lookupStatus(deviceStatus); }

    public int getLoopTime(){return loopTime; }

    public double getFrequency(){
        if (loopTime != 0){
            return 1000000.0/loopTime;
        }
        else {
            return 0;
        }
    }

    public int getEncoderX(){return xEncoderValue; }

    public int getEncoderY(){return yEncoderValue; }

    public double getPosX(){
        return xPosition;
    }

    public double getPosY(){
        return yPosition;
    }

    public double getHeading(){
        double h = (hOrientation+Math.PI/2.0)%(2.0*Math.PI);
        if(h<0) h += 2.0*Math.PI;
        return h;
    }



    public double getVelX(){
        return xVelocity;
    }

    public double getVelY(){
        return yVelocity;
    }

    public double getHeadingVelocity() {
        return hVelocity;
    }

    public Pose2d getPosition(){
        return new Pose2d(xPosition, yPosition, getHeading());
    }

    public Pose2d getVelocity(){
        return new Pose2d(xVelocity, yVelocity, hVelocity);
    }
}



