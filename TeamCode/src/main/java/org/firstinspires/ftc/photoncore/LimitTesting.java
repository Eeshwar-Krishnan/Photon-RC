package org.firstinspires.ftc.photoncore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceDelegate;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetAllDIOInputsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetDIODirectionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetPWMConfigurationCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetPWMEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetPWMPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoConfigurationCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoPulseWidthCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetSingleDIOInputCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;

import static com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand.Channel.CONTROLLER_TEMPERATURE;

@TeleOp
public class LimitTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule module = hardwareMap.getAll(LynxModule.class).get(0);

        LynxUsbDeviceImpl usbDevice;
        RobotUsbDevice device = null;
        Object syncObject = new Object();

        Method getNewNumber = null;

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
            device = (RobotUsbDevice) f2.get(usbDevice);
            Field f3 = usbDevice.getClass().getDeclaredField("engageLock");
            f3.setAccessible(true);
            syncObject = f3.get(usbDevice);

            getNewNumber = module.getClass().getDeclaredMethod("getNewMessageNumber");
            getNewNumber.setAccessible(true);

            Field unfinishedField = module.getClass().getDeclaredField("unfinishedCommands");
            unfinishedField.setAccessible(true);
            unfinishedCommands = (ConcurrentHashMap<Integer, LynxRespondable>) unfinishedField.get(module);
        } catch (IllegalAccessException | NoSuchFieldException | NoSuchMethodException e) {
            e.printStackTrace();
        }

        ArrayList<LynxDekaInterfaceCommand> commands = new ArrayList<>();
        commands.add(new LynxGetADCCommand(module, CONTROLLER_TEMPERATURE, LynxGetADCCommand.Mode.ENGINEERING));

        commands.add(new LynxGetADCCommand(module, CONTROLLER_TEMPERATURE, LynxGetADCCommand.Mode.ENGINEERING));
        commands.add(new LynxGetAllDIOInputsCommand(module));
        commands.add(new LynxGetBulkInputDataCommand(module));
        commands.add(new LynxGetDIODirectionCommand(module, 0));

        commands.add(new LynxGetMotorChannelModeCommand(module, 0));
        commands.add(new LynxGetMotorChannelModeCommand(module, 1));
        commands.add(new LynxGetMotorChannelModeCommand(module, 2));
        commands.add(new LynxGetMotorChannelModeCommand(module, 3));

        commands.add(new LynxGetMotorConstantPowerCommand(module, 0));
        commands.add(new LynxGetMotorConstantPowerCommand(module, 1));
        commands.add(new LynxGetMotorConstantPowerCommand(module, 2));
        commands.add(new LynxGetMotorConstantPowerCommand(module, 3));

        commands.add(new LynxGetMotorEncoderPositionCommand(module, 0));
        commands.add(new LynxGetMotorEncoderPositionCommand(module, 1));
        commands.add(new LynxGetMotorEncoderPositionCommand(module, 2));
        commands.add(new LynxGetMotorEncoderPositionCommand(module, 3));

        commands.add(new LynxGetMotorTargetPositionCommand(module, 0));
        commands.add(new LynxGetMotorTargetPositionCommand(module, 1));
        commands.add(new LynxGetMotorTargetPositionCommand(module, 2));
        commands.add(new LynxGetMotorTargetPositionCommand(module, 3));

        commands.add(new LynxGetMotorTargetVelocityCommand(module, 0));
        commands.add(new LynxGetMotorTargetVelocityCommand(module, 1));
        commands.add(new LynxGetMotorTargetVelocityCommand(module, 2));
        commands.add(new LynxGetMotorTargetVelocityCommand(module, 3));

        commands.add(new LynxGetPWMConfigurationCommand(module));
        commands.add(new LynxGetPWMEnableCommand(module));
        commands.add(new LynxGetPWMPulseWidthCommand(module, 0));
        commands.add(new LynxGetServoConfigurationCommand(module, 0));
        commands.add(new LynxGetServoEnableCommand(module, 0));
        commands.add(new LynxGetServoPulseWidthCommand(module, 0));
        commands.add(new LynxGetSingleDIOInputCommand(module, 0));

        commands.add(new LynxIsMotorAtTargetCommand(module, 0));
        commands.add(new LynxIsMotorAtTargetCommand(module, 1));
        commands.add(new LynxIsMotorAtTargetCommand(module, 2));
        commands.add(new LynxIsMotorAtTargetCommand(module, 3));

        commands.add(new LynxResetMotorEncoderCommand(module, 0));
        commands.add(new LynxResetMotorEncoderCommand(module, 1));
        commands.add(new LynxResetMotorEncoderCommand(module, 2));
        commands.add(new LynxResetMotorEncoderCommand(module, 3));

        commands.add(new LynxSetServoPulseWidthCommand(module, 0, LynxSetServoPulseWidthCommand.apiPulseWidthLast));
        commands.add(new LynxSetServoPulseWidthCommand(module, 1, LynxSetServoPulseWidthCommand.apiPulseWidthLast));
        commands.add(new LynxSetServoPulseWidthCommand(module, 2, LynxSetServoPulseWidthCommand.apiPulseWidthLast));
        commands.add(new LynxSetServoPulseWidthCommand(module, 3, LynxSetServoPulseWidthCommand.apiPulseWidthLast));
        commands.add(new LynxSetServoPulseWidthCommand(module, 4, LynxSetServoPulseWidthCommand.apiPulseWidthLast));
        commands.add(new LynxSetServoPulseWidthCommand(module, 5, LynxSetServoPulseWidthCommand.apiPulseWidthLast));

        commands.add(new LynxSetServoEnableCommand(module, 0, true));
        commands.add(new LynxSetServoEnableCommand(module, 1, true));
        commands.add(new LynxSetServoEnableCommand(module, 2, true));
        commands.add(new LynxSetServoEnableCommand(module, 3, true));
        commands.add(new LynxSetServoEnableCommand(module, 4, true));
        commands.add(new LynxSetServoEnableCommand(module, 5, true));

        commands.add(new LynxSetMotorConstantPowerCommand(module, 0, (int) (LynxSetMotorConstantPowerCommand.apiPowerFirst * 0.1)));
        commands.add(new LynxSetMotorConstantPowerCommand(module, 1, (int) (LynxSetMotorConstantPowerCommand.apiPowerFirst * 0.1)));
        commands.add(new LynxSetMotorConstantPowerCommand(module, 2, (int) (LynxSetMotorConstantPowerCommand.apiPowerFirst * 0.1)));
        commands.add(new LynxSetMotorConstantPowerCommand(module, 3, (int) (LynxSetMotorConstantPowerCommand.apiPowerFirst * 0.1)));

        commands.add(new LynxSetMotorChannelEnableCommand(module, 0, true));
        commands.add(new LynxSetMotorChannelEnableCommand(module, 1, true));
        commands.add(new LynxSetMotorChannelEnableCommand(module, 2, true));
        commands.add(new LynxSetMotorChannelEnableCommand(module, 3, true));

        Byte messageNum = 0;
        ArrayList<Byte> byteArr = new ArrayList<>();
        ArrayList<byte[]> bytesArr = new ArrayList<>();
        for(LynxDekaInterfaceCommand command : commands){
            try {
                messageNum = (Byte) getNewNumber.invoke(module);
            } catch (IllegalAccessException | InvocationTargetException e) {
                e.printStackTrace();
            }

            command.setMessageNumber(messageNum);

            try {
                LynxDatagram datagram = new LynxDatagram(command);
                command.setSerialization(datagram);

                if(command.isAckable() || command.isResponseExpected()){
                    unfinishedCommands.put(command.getMessageNumber(), (LynxRespondable)command);
                }

                datagram.setDestModuleAddress(datagram.getDestModuleAddress() + 1);

                byte[] bytes = datagram.toByteArray();
                for(byte b : bytes) {
                    byteArr.add(b);
                }
                bytesArr.add(bytes);
            } catch (LynxUnsupportedCommandException e) {
                e.printStackTrace();
            }
            messageNum ++;
        }

        byte[] cache = new byte[byteArr.size()];
        for(int i = 0; i < cache.length; i ++){
            cache[i] = byteArr.get(i);
        }

        long start = System.nanoTime();

        synchronized (syncObject) {
            try {
                for(byte[] bArr : bytesArr) {
                    device.write(bArr);
                }
            } catch (InterruptedException | RobotUsbException e) {
                e.printStackTrace();
            }
        }

        long end = System.nanoTime();
        RobotLog.ee("Sent All Commands Somehow????", (end - start)+" | "+byteArr.size());

        int value = 0;
        try {
            Field batteryResponseField = LynxRespondable.class.getDeclaredField("response");
            batteryResponseField.setAccessible(true);

            Field respondedField = LynxRespondable.class.getDeclaredField("isAckOrResponseReceived");
            respondedField.setAccessible(true);
            while(!(Boolean)respondedField.get(commands.get(0))){
                Thread.sleep(5);
            }

            LynxGetADCResponse response = (LynxGetADCResponse) batteryResponseField.get(commands.get(0));
            value = response.getValue();
        } catch (NoSuchFieldException | IllegalAccessException | InterruptedException e) {
            e.printStackTrace();
        }

        RobotLog.e(value+"");
    }
}
