package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.VL53L0XEx;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
//4.54 ms
public class Rev2MTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor");

        VL53L0XEx vl53L0XEx = null;
        try {
            I2cDeviceSynch device = (I2cDeviceSynch) ReflectionUtils.getField(distanceSensor.getClass(), "deviceClient").get(distanceSensor);
            vl53L0XEx = new VL53L0XEx(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        waitForStart();

        long start = System.currentTimeMillis();
        for(int i = 0; i < 50; i ++){
            distanceSensor.getDistance(DistanceUnit.MM);
        }
        long dt = System.currentTimeMillis() - start;

        long start2 = System.currentTimeMillis();
        for(int i = 0; i < 50; i ++){
            vl53L0XEx.getDistance(DistanceUnit.MM);
        }
        long dt2 = System.currentTimeMillis() - start2;

        while (opModeIsActive()){
            telemetry.addData("Average Time Stock", ((dt) / 50.0) + " ms");
            telemetry.addData("Average Time Photon", ((dt2) / 50.0) + " ms");
            telemetry.update();
        }
    }
}
