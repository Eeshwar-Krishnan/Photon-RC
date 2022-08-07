package org.firstinspires.ftc.photoncore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp
public class RTCTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        I2cDeviceSynch deviceSync = hardwareMap.i2cDeviceSynch.get("rtc");

        deviceSync.write8(16);
        DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get("3");
        Encoder encoder = new Encoder(motor);
        telemetry.setMsTransmissionInterval(15);
        while (!isStopRequested()){
            telemetry.addData("Pos", encoder.getCurrentPosition());
            telemetry.addData("Vel", encoder.getRawVelocity());
            telemetry.addData("Vel Corr", encoder.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
