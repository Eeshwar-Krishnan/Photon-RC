package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.VL53L0XEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
//2.88 ms
public class RevColorSensorV3Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 distanceSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        telemetry.setMsTransmissionInterval(15);

        while(!isStarted()){
            telemetry.addData("Distance REV", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        long start = System.currentTimeMillis();
        for(int i = 0; i < 50; i ++){
            RobotLog.ii("ColorSensorTest", ""+distanceSensor.getDistance(DistanceUnit.MM));
        }
        long dt = System.currentTimeMillis() - start;

        RevColorSensorV3Ex revColorSensorV3Ex = null;
        try {
            I2cDeviceSynchSimple device = (I2cDeviceSynchSimple) ReflectionUtils.getField(distanceSensor.getClass(), "deviceClient").get(distanceSensor);
            revColorSensorV3Ex = new RevColorSensorV3Ex(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        long start2 = System.currentTimeMillis();
        for(int i = 0; i < 50; i ++){
            RobotLog.ii("ColorSensorTestEx", ""+revColorSensorV3Ex.getDistance(DistanceUnit.MM));
        }
        long dt2 = System.currentTimeMillis() - start2;

        while (opModeIsActive()){
            telemetry.addData("Average Time Stock", ((dt) / 50.0) + " ms");
            telemetry.addData("Average Time Photon", ((dt2) / 50.0) + " ms");
            telemetry.update();
        }

        while (opModeIsActive()){
            telemetry.addData("Distance PHOTON", revColorSensorV3Ex.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
