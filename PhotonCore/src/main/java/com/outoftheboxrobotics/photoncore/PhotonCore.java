package com.outoftheboxrobotics.photoncore;

import android.content.Context;


import com.outoftheboxrobotics.photoncore.Neutrino.BNO055.BNO055ImuEx;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceDelegate;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadSingleByteCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class PhotonCore implements Runnable, OpModeManagerNotifier.Notifications {
    protected static final PhotonCore instance = new PhotonCore();
    protected AtomicBoolean enabled, threadEnabled;

    private List<LynxModule> modules;
    private Thread thisThread = null;
    private Object syncLock;

    private final Object messageSync = new Object();

    private RobotUsbDevice robotUsbDevice;
    private HashMap<LynxModule, RobotUsbDevice> usbDeviceMap;

    public static LynxModule CONTROL_HUB, EXPANSION_HUB;

    private OpModeManagerImpl opModeManager;

    private ArrayList<LynxModule> toGyroCache;
    private HashMap<LynxModule, Semaphore> bus0Locks;

    public static class ExperimentalParameters{
        private final AtomicBoolean singlethreadedOptimized = new AtomicBoolean(true);
        private final AtomicInteger maximumParallelCommands = new AtomicInteger(4);

        public void setSinglethreadedOptimized(boolean state){
            this.singlethreadedOptimized.set(state);
        }

        public boolean setMaximumParallelCommands(int maximumParallelCommands){
            if(maximumParallelCommands <= 0){
                return false;
            }
            this.maximumParallelCommands.set(maximumParallelCommands);
            return true;
        }
    }

    public static ExperimentalParameters experimental = new ExperimentalParameters();

    public PhotonCore(){
        CONTROL_HUB = null;
        EXPANSION_HUB = null;
        enabled = new AtomicBoolean(false);
        threadEnabled = new AtomicBoolean(false);
        usbDeviceMap = new HashMap<>();
        toGyroCache = new ArrayList<>();
        bus0Locks = new HashMap<>();
    }

    public static void enable(){
        instance.enabled.set(true);
        //Might as well turn on bulk caching for the user, no penalty for doing this
        if(CONTROL_HUB.getBulkCachingMode() == LynxModule.BulkCachingMode.OFF){
            CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        if(EXPANSION_HUB != null && EXPANSION_HUB.getBulkCachingMode() == LynxModule.BulkCachingMode.OFF){
            EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private boolean enableGyroCache(LynxModule module){
        if(module == null){
            return false;//Most likely an unconnected hub, fail fast
        }
        if(toGyroCache.contains(module)){
            return false; //Already being cached, ignore
        }
        toGyroCache.add(module);
        bus0Locks.put(module, new Semaphore(1)); //Only one device can be accessed per bus according to REV
        return true;
    }

    private int getI2CPort(LynxCommand command){
        if(command instanceof LynxI2cWriteSingleByteCommand ||
                command instanceof LynxI2cWriteMultipleBytesCommand ||
                command instanceof LynxI2cReadSingleByteCommand ||
                command instanceof LynxI2cReadMultipleBytesCommand ||
                command instanceof LynxI2cReadStatusQueryCommand ||
                command instanceof LynxI2cWriteStatusQueryCommand){
            try {
                return (int) ReflectionUtils.getField(command.getClass(), "i2cBus").get(command);//All these commands share the same variable name for the i2c bus, very convenient for reflection
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        return -1;//Not an i2c command, ignore
    }

    public static void disable(){
        instance.enabled.set(false);
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(instance);
        instance.opModeManager = eventLoop.getOpModeManager();
    }

    public static boolean aquireBus0Lock(LynxModule module) throws InterruptedException {
        if(!instance.bus0Locks.containsKey(module)){
            return false;
        }
        instance.bus0Locks.get(module).acquire();
        return true;
    }

    public static boolean releaseBus0Lock(LynxModule module) throws InterruptedException {
        if(!instance.bus0Locks.containsKey(module)){
            return false;
        }
        instance.bus0Locks.get(module).release();
        return true;
    }

    protected static boolean registerSend(LynxCommand command) throws LynxUnsupportedCommandException, InterruptedException {

        PhotonLynxModule photonModule = (PhotonLynxModule) command.getModule();

        if(!instance.usbDeviceMap.containsKey(photonModule)){
            return false;//RS485 hub, can't parallelize, get out of dodge to avoid adding extra overhead
        }

        synchronized (instance.messageSync) {
            while (((PhotonLynxModule)photonModule).getUnfinishedCommands().size() > experimental.maximumParallelCommands.get()){
                //We can only parallelize so many commands before the hub runs out of memory space, usually around 7 or 8 commands
            }

            if(!experimental.singlethreadedOptimized.get()) {
                boolean noSimilar = false;
                while (!noSimilar) {
                    noSimilar = true;
                    for (LynxRespondable respondable : photonModule.getUnfinishedCommands().values()) {
                        if (instance.isSimilar(respondable, command)) {
                            noSimilar = false;//Don't allow similar commands, this allows threads to have somewhat equal priorities when using the send system
                        }
                    }
                }
            }

            byte messageNum = photonModule.getNewMessageNumber();

            command.setMessageNumber(messageNum);

            try {
                LynxDatagram datagram = new LynxDatagram(command);
                command.setSerialization(datagram);

                if (command.isAckable() || command.isResponseExpected()) {
                    photonModule.getUnfinishedCommands().put(command.getMessageNumber(), (LynxRespondable) command);
                }

                byte[] bytes = datagram.toByteArray();

                double msLatency = 0;
                synchronized (instance.syncLock) {
                    long start = System.nanoTime();
                    instance.usbDeviceMap.get(photonModule).write(bytes);
                    long stop = System.nanoTime();
                    msLatency = (stop - start) * 1.0e-6;
                }
                //RobotLog.ii("PhotonCore", "Wrote " + bytes.length + " bytes " + photonModule.getUnfinishedCommands().size() + " | " + (msLatency));

                if (shouldAckImmediately(command)) {
                    command.onAckReceived(new LynxAck(photonModule, false));//Ack immediately because we really don't need to wait for an ack for this command
                }
            } catch (LynxUnsupportedCommandException | RobotUsbException e) {
                e.printStackTrace();
            }
        }

        return true;
    }

    protected static boolean shouldParallelize(LynxCommand command){
        return (command instanceof LynxSetMotorConstantPowerCommand) ||
                (command instanceof LynxSetServoPulseWidthCommand);//Only two tested commands at the moment, more could theoretically be added
    }

    protected static boolean shouldAckImmediately(LynxCommand command){
        return (command instanceof LynxSetMotorConstantPowerCommand) ||
                (command instanceof LynxSetServoPulseWidthCommand);//Basically no side effects from ignoring the response with modern comms, nacks are very very rare
    }

    private boolean isSimilar(LynxRespondable respondable1, LynxRespondable respondable2){
        return (respondable1.getDestModuleAddress() == respondable2.getDestModuleAddress()) &&
                (respondable1.getCommandNumber() == respondable2.getCommandNumber());
    }

    protected static LynxMessage getCacheResponse(LynxCommand command){
        return null;//Legacy, ignored
    }

    @Override
    public void run() {
        while(threadEnabled.get()){
            //Legacy, ignored
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if(opModeManager.getActiveOpModeName().equals(OpModeManager.DEFAULT_OP_MODE_NAME)){
            return; //Don't waste time setting up photon when the opmode is stopped
        }

        HardwareMap map = opMode.hardwareMap;

        toGyroCache.clear();
        bus0Locks.clear();

        boolean replacedPrev = false;
        boolean hasChub = false;
        for(LynxModule module : map.getAll(LynxModule.class)){
            if(module instanceof PhotonLynxModule){
                replacedPrev = true;
            }
            if(LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())){
                hasChub = true;
            }
        }
        if(replacedPrev){//Seems this has already been run and there are already photonlynxmodule objects, we need to remove the new lynxmodule objects made by the sdk
            HashMap<String, HardwareDevice> toRemove = new HashMap<>();
            for(LynxModule module : map.getAll(LynxModule.class)){
                if(!(module instanceof PhotonLynxModule)){
                    toRemove.put((String) map.getNamesOf(module).toArray()[0], module);
                }
            }
            for(String s : toRemove.keySet()){
                map.remove(s, toRemove.get(s));
            }
        }else{
            CONTROL_HUB = null;
            EXPANSION_HUB = null;
        }

        instance.modules = map.getAll(LynxModule.class);
        ArrayList<String> moduleNames = new ArrayList<>();
        HashMap<LynxModule, PhotonLynxModule> replacements = new HashMap<>();
        for(LynxModule module : instance.modules){
            moduleNames.add((String) map.getNamesOf(module).toArray()[0]);
        }
        LynxUsbDeviceImpl usbDevice = null, usbDevice1 = null;
        for(String s : moduleNames){
            LynxModule module = (LynxModule) map.get(LynxModule.class, s);
            if(module instanceof PhotonLynxModule){
                continue;
            }
            try {
                PhotonLynxModule photonLynxModule = new PhotonLynxModule(
                        (LynxUsbDevice) ReflectionUtils.getField(module.getClass(), "lynxUsbDevice").get(module),
                        (Integer)ReflectionUtils.getField(module.getClass(), "moduleAddress").get(module),
                        (Boolean)ReflectionUtils.getField(module.getClass(), "isParent").get(module),
                        (Boolean)ReflectionUtils.getField(module.getClass(), "isUserModule").get(module)
                );
                RobotLog.ee("PhotonCoreLynxNames", s);
                ReflectionUtils.deepCopy(module, photonLynxModule);
                map.remove(s, module);
                map.put(s, photonLynxModule);
                replacements.put(module, photonLynxModule);

                if(module.isParent() && (hasChub && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())) && CONTROL_HUB == null){
                    CONTROL_HUB = photonLynxModule;

                    ConcurrentHashMap<Integer, LynxRespondable> unfinishedCommands = new ConcurrentHashMap<>();
                    try {
                        Field f1 = module.getClass().getDeclaredField("lynxUsbDevice");
                        f1.setAccessible(true);
                        LynxUsbDevice tmp = (LynxUsbDevice) f1.get(module);
                        if(tmp instanceof LynxUsbDeviceDelegate){
                            Field tmp2 = LynxUsbDeviceDelegate.class.getDeclaredField("delegate");
                            tmp2.setAccessible(true);
                            usbDevice = (LynxUsbDeviceImpl) tmp2.get(tmp);
                        }else{
                            usbDevice = (LynxUsbDeviceImpl) tmp;
                        }
                        Field f2 = usbDevice.getClass().getSuperclass().getDeclaredField("robotUsbDevice");
                        f2.setAccessible(true);
                        Field f3 = usbDevice.getClass().getDeclaredField("engageLock");
                        f3.setAccessible(true);
                        syncLock = f3.get(usbDevice);

                        robotUsbDevice = (RobotUsbDevice) f2.get(usbDevice);
                        usbDeviceMap.put(photonLynxModule, robotUsbDevice);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    } catch(NoSuchFieldException e){
                        e.printStackTrace();
                    }
                }else{
                    if(module.isParent()) {
                        try {
                            Field f1 = module.getClass().getDeclaredField("lynxUsbDevice");
                            f1.setAccessible(true);
                            LynxUsbDevice tmp = (LynxUsbDevice) f1.get(module);
                            if (tmp instanceof LynxUsbDeviceDelegate) {
                                Field tmp2 = LynxUsbDeviceDelegate.class.getDeclaredField("delegate");
                                tmp2.setAccessible(true);
                                usbDevice = (LynxUsbDeviceImpl) tmp2.get(tmp);
                            } else {
                                usbDevice = (LynxUsbDeviceImpl) tmp;
                            }
                            Field f2 = usbDevice.getClass().getSuperclass().getDeclaredField("robotUsbDevice");
                            f2.setAccessible(true);
                            Field f3 = usbDevice.getClass().getDeclaredField("engageLock");
                            f3.setAccessible(true);
                            syncLock = f3.get(usbDevice);

                            robotUsbDevice = (RobotUsbDevice) f2.get(usbDevice);
                            usbDeviceMap.put(photonLynxModule, robotUsbDevice);
                        } catch (NoSuchFieldException e) {
                            e.printStackTrace();
                        }
                    }
                    EXPANSION_HUB = photonLynxModule;
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        for(LynxModule m : replacements.keySet()){
            usbDevice.removeConfiguredModule(m);
            try {
                usbDevice.addConfiguredModule(replacements.get(m));
            } catch (InterruptedException e) {
                e.printStackTrace();
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
        }

        HashMap<String, HardwareDevice> replacedNeutrino = new HashMap<>(), removedNeutrino = new HashMap<>();
        for(HardwareDevice device : map.getAll(HardwareDevice.class)){
            if(!(device instanceof LynxModule)){
                RobotLog.i(map.getNamesOf(device).toArray()[0].toString());
                if(device instanceof I2cDeviceSynchDevice){
                    try {
                        I2cDeviceSynchSimple device2 = (I2cDeviceSynchSimple) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                        if(!(device2 instanceof LynxI2cDeviceSynch)){
                            device2 = (I2cDeviceSynchSimple) ReflectionUtils.getField(device2.getClass(), "i2cDeviceSynchSimple").get(device2);
                        }
                        setLynxObject(device2, replacements);
                        RobotLog.e("" + (device2 instanceof LynxI2cDeviceSynch));
                    } catch (Exception ignored) {
                    }
                }else if (device instanceof I2cDeviceSynchSimple){
                    try {
                        I2cDeviceSynchSimple device2 = (I2cDeviceSynchSimple) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                        setLynxObject(device2, replacements);
                    } catch (Exception ignored) {
                    }
                }else {
                    setLynxObject(device, replacements);
                }
                if(device instanceof Rev2mDistanceSensor){
                    I2cDeviceSynch tmp = null;
                    try {
                        tmp = (I2cDeviceSynch) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                    Rev2mDistanceSensorEx vl53L0XEx = new Rev2mDistanceSensorEx(tmp);
                    replacedNeutrino.put((String) map.getNamesOf(device).toArray()[0], vl53L0XEx);
                    removedNeutrino.put((String) map.getNamesOf(device).toArray()[0], device);
                }
                if(device instanceof RevColorSensorV3){
                    RevColorSensorV3Ex revColorSensorV3Ex;
                    try {
                        I2cDeviceSynchSimple tmp = (I2cDeviceSynchSimple) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                        revColorSensorV3Ex = new RevColorSensorV3Ex(tmp);
                        replacedNeutrino.put((String) map.getNamesOf(device).toArray()[0], revColorSensorV3Ex);
                        removedNeutrino.put((String) map.getNamesOf(device).toArray()[0], device);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }
                if(device instanceof BNO055IMU){
                    BNO055ImuEx bno055ImuEx;
                    try {
                        I2cDeviceSynch tmp = (I2cDeviceSynch) ReflectionUtils.getField(device.getClass(), "deviceClient").get(device);
                        bno055ImuEx = new BNO055ImuEx(tmp);
                        replacedNeutrino.put((String) map.getNamesOf(device).toArray()[0], bno055ImuEx);
                        removedNeutrino.put((String) map.getNamesOf(device).toArray()[0], device);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        for(String s : replacedNeutrino.keySet()){
            map.remove(s, removedNeutrino.get(s));
            map.put(s, replacedNeutrino.get(s));
        }

        if(thisThread == null || !thisThread.isAlive()){
            thisThread = new Thread(this);
            threadEnabled.set(true);
            thisThread.start();
        }
    }

    private void setLynxObject(Object device, HashMap<LynxModule, PhotonLynxModule> replacements){
        Field f = ReflectionUtils.getField(device.getClass(), LynxModule.class);
        if (f != null) {
            f.setAccessible(true);
            try {
                LynxModule module = (LynxModule) f.get(device);
                if (module == null) {
                    return;
                }
                if (replacements.containsKey(module)) {
                    f.set(device, replacements.get(module));
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        enabled.set(false);
        threadEnabled.set(false);
    }
}
