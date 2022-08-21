package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.Neutrino.I2cCacheProfiles.InternalIMUCacheProfile;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
//4.38 ms
public class IMUTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");

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
        //((BNO055ImuEx) imu).enableGyroCaching();
        PhotonCore.experimental.addI2cCache(PhotonCore.CONTROL_HUB, new InternalIMUCacheProfile(parameters));
        PhotonCore.enable();
        long start2 = System.currentTimeMillis();
        for(int i = 0; i < 500; i ++){
            RobotLog.ii("IMU Tester 2", ""+imu.getAngularOrientation().firstAngle);
        }
        long dt2 = System.currentTimeMillis() - start2;

        telemetry.setMsTransmissionInterval(15);
        long now = System.nanoTime();
        double dt3 = 1;
        ((RevColorSensorV3Ex)sensor).setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        PhotonCore.experimental.addI2cCache(PhotonCore.CONTROL_HUB, new PhotonCore.I2CReadCache(1, I2cAddr.create7bit(0x52), 0x0D, 9));
        while (!isStarted()){
            telemetry.addData("Average Time Stock", ((dt) / 500.0) + " ms");
            telemetry.addData("Average Time Photon", ((dt2) / 500.0) + " ms");
            telemetry.addData("IMU angle", imu.getAngularOrientation().firstAngle);
            long read = imu.getAngularOrientation().acquisitionTime;
            if(read != now) {
                dt3 = (read - now) / 1.0E9;
                now = read;
            }
            telemetry.addData("Update Rate (HZ)", 1.0/dt3);
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        waitForStart();
    }
}
