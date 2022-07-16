package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
//4.38 ms
public class IMUTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523;

        imu.initialize(parameters);

        long start = System.currentTimeMillis();
        for(int i = 0; i < 500; i ++){
            RobotLog.ii("IMU Tester 1", ""+imu.getAngularOrientation().firstAngle);
        }
        long dt = System.currentTimeMillis() - start;
        PhotonCore.enable();
        long start2 = System.currentTimeMillis();
        for(int i = 0; i < 500; i ++){
            RobotLog.ii("IMU Tester 2", ""+imu.getAngularOrientation().firstAngle);
        }
        long dt2 = System.currentTimeMillis() - start2;

        while (!isStarted()){
            telemetry.addData("Average Time Stock", ((dt) / 500.0) + " ms");
            telemetry.addData("Average Time Photon", ((dt2) / 500.0) + " ms");
            telemetry.update();
        }

        waitForStart();
    }
}
