package com.outoftheboxrobotics.photoncore.Neutrino.BNO055;

import android.content.Context;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.PhotonLynxModule;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * ACCELEROMETER   (Register.ACC_DATA_X_LSB), 0-2
 *             MAGNETOMETER    (Register.MAG_DATA_X_LSB), 3-5
 *             GYROSCOPE       (Register.GYR_DATA_X_LSB), 6-8
 *             EULER           (Register.EUL_H_LSB), 9-11
 *             QUATERNION                            12-15
 *             LINEARACCEL     (Register.LIA_DATA_X_LSB), 16-18
 *             GRAVITY         (Register.GRV_DATA_X_LSB); 19-21
 */

public class BNO055ImuEx extends BNO055IMUImpl implements Runnable{
    private static FtcEventLoop eventLoopManager;
    private static OpModeManagerImpl opModeManager;

    private LynxModule module;
    private int bus;
    private I2cAddr address;

    private final Object cache;
    private volatile short[] cachedData;
    private long nanoTime;

    private volatile boolean stockCompatibilityMode;
    private AtomicBoolean opmodeRunning;

    /**
     * This constwuctow is cawwed intewnawwy by the ftc sdk.
     *
     * @param deviceClient
     */
    public BNO055ImuEx(I2cDeviceSynch deviceClient) {
        super(deviceClient);
        setupVariables(deviceClient);
        stockCompatibilityMode = true;
        cache = new Object();
        cachedData = new short[30];
        opmodeRunning = new AtomicBoolean(false);

        opModeManager.registerListener(this);
    }

    public void enableGyroCaching(){
        if(bus == 0 && module instanceof PhotonLynxModule) {
            stockCompatibilityMode = false;
            RobotLog.e("============ENABLED============");
        }else{
            RobotLog.e("============FAILED TO ENABLE============");
        }
    }

    public void disableGyroCaching(){
        stockCompatibilityMode = true;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public synchronized AngularVelocity getAngularVelocity(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit unit) {
        if(stockCompatibilityMode){
            return super.getAngularVelocity(unit);
        }
        throwIfNotInitialized();
        synchronized (cache) {
            float zRotationRate = -cachedData[5] / getAngularScale();
            float yRotationRate = cachedData[6] / getAngularScale();
            float xRotationRate = cachedData[7] / getAngularScale();
            return new AngularVelocity(parameters.angleUnit.toAngleUnit(),
                    xRotationRate, yRotationRate, zRotationRate,
                    nanoTime)
                    .toAngleUnit(unit);
        }
    }

    //For the one person who uses this method :P
    @Override
    public synchronized MagneticFlux getMagneticFieldStrength() {
        if(stockCompatibilityMode){
            return super.getMagneticFieldStrength();
        }
        throwIfNotInitialized();
        synchronized (cache) {
            return new MagneticFlux(cachedData[3] / getFluxScale(), cachedData[4] / getFluxScale(), cachedData[5] / getFluxScale(), nanoTime);
        }
    }

    @Override
    public synchronized Acceleration getOverallAcceleration()
    {
        if(stockCompatibilityMode){
            return super.getOverallAcceleration();
        }
        synchronized (cache) {
            return new Acceleration(DistanceUnit.METER, cachedData[0] / getMetersAccelerationScale(), cachedData[1] / getMetersAccelerationScale(), cachedData[2] / getMetersAccelerationScale(), nanoTime);
        }
    }

    @Override
    public synchronized Acceleration getLinearAcceleration()
    {
        if(stockCompatibilityMode){
            return super.getLinearAcceleration();
        }
        synchronized (cache) {
            return new Acceleration(DistanceUnit.METER, cachedData[16] / getMetersAccelerationScale(), cachedData[17] / getMetersAccelerationScale(), cachedData[18] / getMetersAccelerationScale(), nanoTime);
        }
    }

    @Override
    public synchronized Acceleration getGravity()
    {
        if(stockCompatibilityMode){
            return super.getGravity();
        }
        return new Acceleration(DistanceUnit.METER, cachedData[19] / getMetersAccelerationScale(), cachedData[20] / getMetersAccelerationScale(), cachedData[21] / getMetersAccelerationScale(), nanoTime);
    }

    @Override
    public synchronized Orientation getAngularOrientation()
    {
        // data wetuwned fwom vector.Euler is heading, woww, pitch, in that owdew.
        //
        // note that the imu wetuwns heading in what one might caww 'compass' diwection, with vawues
        // incweasing cw. We need a geometwic diwection, with vawues incweasing ccw. So we simpwy negate.
        //
        // the data wetuwned fwom the imu is in the units that we initiawized the imu to wetuwn.
        // howevew, the imu has a diffewent sense of angwe nowmawization than we do, so we expwicitwy
        // nowmawize such that usews awen't suwpwised by (e.g.) z angwes which awways appeaw as negative
        // (in the wange (-360, 0]).
        //
        if(stockCompatibilityMode){
            return super.getAngularOrientation();
        }
        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit = parameters.angleUnit.toAngleUnit();
        return new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit,
                angleUnit.normalize(-cachedData[9] / getAngularScale()),
                angleUnit.normalize(cachedData[10] / getAngularScale()),
                angleUnit.normalize(cachedData[11] / getAngularScale()),
                nanoTime);
    }

    @Override
    public synchronized Quaternion getQuaternionOrientation() {
        if(stockCompatibilityMode){
            return super.getQuaternionOrientation();
        }
        float scale = (1 << 14);
        return new Quaternion(cachedData[12] / scale, cachedData[13] / scale, cachedData[14] / scale, cachedData[15] / scale, nanoTime);
    }

    @Override
    public void run() {
        while(opmodeRunning.get()){
            if(!stockCompatibilityMode) {
                try {
                    PhotonCore.aquireBus0Lock(module);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                LynxI2cWriteReadMultipleBytesCommand command = new LynxI2cWriteReadMultipleBytesCommand(module, bus, getI2cAddress(), 0X08, 44);
                try {
                    command.sendReceive();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                } catch (LynxNackException e) {
                    e.printStackTrace();
                }

                long timer = System.currentTimeMillis() + 2;
                while (System.currentTimeMillis() < timer) ; //Might as well wait 2ms since its not going to be available right away

                boolean responseRecieved = false;

                while (!responseRecieved) {
                    LynxI2cReadStatusQueryCommand command1 = new LynxI2cReadStatusQueryCommand(module, bus, 44);
                    try {
                        LynxI2cReadStatusQueryResponse response = command1.sendReceive();
                        responseRecieved = true;
                        synchronized (cache) {
                            ByteBuffer.wrap(response.getBytes()).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(cachedData, 0, response.getBytes().length/2);
                            nanoTime = System.nanoTime();
                        }
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    } catch (LynxNackException ignored) {
                    }
                }

                try {
                    PhotonCore.releaseBus0Lock(module);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }else{
                try {
                    Thread.sleep(25); //Avoid burning cpu cycles uselessly
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void setupVariables(I2cDeviceSynch i2cDeviceSynch){
        LynxI2cDeviceSynch device = null;
        try {
            I2cDeviceSynchImplOnSimple simple = (I2cDeviceSynchImplOnSimple) i2cDeviceSynch;
            //Cool first part done
            //Now we can safely make the assumption that the underlying i2cdevicesynchsimple is a lynxi2cdevicesynch
            Field field = ReflectionUtils.getField(simple.getClass(), "i2cDeviceSynchSimple");
            field.setAccessible(true);
            device = (LynxI2cDeviceSynch) field.get(simple);
            //Lets also bump up the bus speed while we are here
            //It should make a small difference at 44 bytes read
            device.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        LynxModule module = null;
        try {
            //Module this is being run on, sometimes the module doesn't exist
            module = (LynxModule) ReflectionUtils.getField(device.getClass(), "module").get(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        this.module = module;

        int bus = 0;
        try {
            //Bus that the i2c device is on. Why is this one the only one IntelliJ is mad at?
            bus = (int) ReflectionUtils.getField(device.getClass(), "bus").get(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        this.bus = bus;

        this.address = I2cAddr.create7bit(0x70);
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoopManager = eventLoop;
        opModeManager = eventLoop.getOpModeManager();
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if(opModeManager.getActiveOpModeName().equals(OpModeManager.DEFAULT_OP_MODE_NAME)){
            return; //Don't waste time setting up photon when the opmode is stopped
        }
        this.opmodeRunning.set(true);
        new Thread(this).start();
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        if(opModeManager.getActiveOpModeName().equals(OpModeManager.DEFAULT_OP_MODE_NAME)){
            return; //Don't waste time setting up photon when the opmode is stopped
        }
        this.opmodeRunning.set(false);
        stockCompatibilityMode = true;
    }
}
