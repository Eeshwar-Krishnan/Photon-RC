package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class MB1242Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MB1242Ex mb1242Ex = hardwareMap.get(MB1242Ex.class, "sonic");
        telemetry.setMsTransmissionInterval(25);
        long prev = System.currentTimeMillis();
        while(!isStarted()){
            long now = System.currentTimeMillis();
            telemetry.addData("Distance", mb1242Ex.getDistance(DistanceUnit.MM));
            long dt = now - prev;
            prev = now;
            telemetry.addData("Interval (ms)", dt);
            telemetry.update();
        }
        while(opModeIsActive()){
            long now = System.currentTimeMillis();
            telemetry.addData("Distance", mb1242Ex.getDistanceAsync(DistanceUnit.MM));
            long dt = now - prev;
            prev = now;
            telemetry.addData("Interval (ms)", dt);
            telemetry.update();
        }
    }
}
