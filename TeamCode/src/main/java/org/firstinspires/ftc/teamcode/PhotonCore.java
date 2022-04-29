package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.MessageKeyedLock;
import com.qualcomm.hardware.lynx.MultiMessageKeyedLock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public class PhotonCore {
    private static PhotonCore instance = null;

    public static PhotonCore getInstance(){
        if(instance == null) {
            instance = new PhotonCore();
        }
        return instance;
    }

    private ArrayList<MultiMessageKeyedLock> discoveredLocks;
    private ArrayList<Thread> threads;
    private ArrayList<PhotonThread> photonThreads;

    public static void setup(HardwareMap map, List<PhotonThread> threads){
        getInstance().discoveredLocks = new ArrayList<>();
        getInstance().threads = new ArrayList<>();
        getInstance().photonThreads = new ArrayList<>();

        List<LynxModule> modules = map.getAll(LynxModule.class);
        for(LynxModule module : modules){
            try {
                Field f = module.getClass().getDeclaredField("lynxUsbDevice");
                f.setAccessible(true);
                LynxUsbDeviceImpl device = (LynxUsbDeviceImpl)((LynxUsbDevice) f.get(module));
                Field f2 = device.getClass().getDeclaredField("networkTransmissionLock");
                f2.setAccessible(true);
                MessageKeyedLock childLock = (MessageKeyedLock) f2.get(device);

                if(childLock instanceof MultiMessageKeyedLock){
                    continue;
                }

                MultiMessageKeyedLock multiMessageKeyedLock = new MultiMessageKeyedLock("lynx xmit lock", 500);
                getInstance().discoveredLocks.add(multiMessageKeyedLock);
                multiMessageKeyedLock.passNewLock(childLock);

                f2.set(device, multiMessageKeyedLock);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        for(PhotonThread photonThread : threads){
            getInstance().threads.add(new Thread(photonThread));
        }

        getInstance().photonThreads.addAll(threads);

        for(MultiMessageKeyedLock lock : getInstance().discoveredLocks){
            lock.addThreadpool(getInstance().threads);
        }

        for(Thread thread : getInstance().threads){
            thread.start();
        }
    }
}
