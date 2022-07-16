package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxInjectDataLogHintCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetDebugLogLevelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BulkInputDataTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(PhotonCore.CONTROL_HUB);
        try {
            command.sendReceive();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }
    }
}
