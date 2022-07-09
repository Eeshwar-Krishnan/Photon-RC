/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.qualcomm.hardware.lynx;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.standard.LynxKeepAliveCommand;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


public class MultiMessageKeyedLock extends MessageKeyedLock{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    private final List<MessageKeyedLock> locks;
    private final ConcurrentHashMap<Thread, MessageKeyedLock> assignedLocks;
    private volatile MessageKeyedLock globalLock;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public MultiMessageKeyedLock(String name)
        {
        this(name, 500);
        }

    public MultiMessageKeyedLock(String name, int msAquisitionTimeout) {
        super(name, msAquisitionTimeout);
        locks = Collections.synchronizedList(new ArrayList<MessageKeyedLock>());

        for(int i = 0; i < 7; i ++){
            locks.add(new MessageKeyedLock(name, msAquisitionTimeout));
        }

        assignedLocks = new ConcurrentHashMap<>();
        globalLock = new MessageKeyedLock(name, msAquisitionTimeout);

    }

    @Override
    public void reset() throws InterruptedException {
        globalLock.reset();
        for(MessageKeyedLock lock : locks){
            lock.reset();
        }
    }

    @Override
    public void acquire(@NonNull LynxMessage message) throws InterruptedException {
        if(assignedLocks.containsKey(Thread.currentThread())){
            assignedLocks.get(Thread.currentThread()).acquire(message);
        }else{
            globalLock.acquire(message);
        }
    }

    @Override
    public void release(@NonNull LynxMessage message) throws InterruptedException {
        if(assignedLocks.containsKey(Thread.currentThread())){
            assignedLocks.get(Thread.currentThread()).acquire(message);
        }else{
            globalLock.acquire(message);
        }
    }

    @Override
    public void lockAcquisitions() {
        globalLock.lockAcquisitions();
        for(MessageKeyedLock lock : locks){
            lock.lockAcquisitions();
        }
    }

    @Override
    public void throwOnLockAcquisitions(boolean shouldthrow) {
        globalLock.throwOnLockAcquisitions(shouldthrow);
        for(MessageKeyedLock lock : locks){
            lock.throwOnLockAcquisitions(shouldthrow);
        }
    }

    public void addThreadpool(ArrayList<Thread> threads){
        for(int i = 0; (i < (locks.size()) && i < (threads.size())); i ++){
            assignedLocks.put(threads.get(i), locks.get(i));
        }
    }

    public void passNewLock(MessageKeyedLock lock){
        this.globalLock = lock;
    }
}
