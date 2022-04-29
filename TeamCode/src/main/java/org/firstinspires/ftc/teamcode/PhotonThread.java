package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.qual.A;

import java.util.concurrent.atomic.AtomicBoolean;

public interface PhotonThread extends Runnable {
    AtomicBoolean isStopRequested = new AtomicBoolean(false);
    @Override
    default void run() {
        init();
        while(!isStopRequested.get()){
            loop();
        }
    }

    void loop();

    default void init(){

    }

    default void stop(){
        isStopRequested.set(true);
    }
}
