package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ManualEncoderVelTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motorEx = (DcMotorEx) hardwareMap.dcMotor.get("3");
        Encoder encoder = new Encoder(motorEx);

        double lastPos = 0;
        long lastNs = 0;
        telemetry.setMsTransmissionInterval(15);
        while (!isStopRequested()){
            encoder.getCurrentPosition();
            LynxGetMotorEncoderPositionCommand command = new LynxGetMotorEncoderPositionCommand(PhotonCore.CONTROL_HUB, 3);
            LynxGetMotorEncoderPositionResponse response = null;
            try {
                 response = command.sendReceive();
            } catch (LynxNackException e) {
                e.printStackTrace();
            }
            double pos = response.getPosition();
            long timestamp = response.getPayloadTimeWindow().getNanosecondsFirst();
            double dt = (timestamp - lastNs) / 1.0E9;
            double vel = (pos - lastPos) / dt;
            lastPos = pos;
            lastNs = timestamp;

            telemetry.addData("User vel", vel);
            telemetry.addData("Corr vel", encoder.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
