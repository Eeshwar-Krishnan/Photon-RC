package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.VL53L0XEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.ReflectionUtils;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor");

        VL53L0XEx vl53L0XEx = null;
        try {
            I2cDeviceSynch device = (I2cDeviceSynch) ReflectionUtils.getField(distanceSensor.getClass(), "deviceClient").get(distanceSensor);
            vl53L0XEx = new VL53L0XEx(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        RevColorSensorV3 distanceSensor2 = hardwareMap.get(RevColorSensorV3.class, "color");

        RevColorSensorV3Ex revColorSensorV3Ex = null;
        try {
            I2cDeviceSynchSimple device = (I2cDeviceSynchSimple) ReflectionUtils.getField(distanceSensor2.getClass(), "deviceClient").get(distanceSensor2);
            revColorSensorV3Ex = new RevColorSensorV3Ex(device);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        MB1242Ex mb1242Ex = hardwareMap.get(MB1242Ex.class, "sonic");

        boolean enabled = false;

        LynxDcMotorController motorController = (LynxDcMotorController) hardwareMap.dcMotorController.iterator().next();
        DcMotorEx motor0 = new DcMotorImplEx(motorController, 0);
        DcMotorEx motor1 = new DcMotorImplEx(motorController, 1);
        DcMotorEx motor2 = new DcMotorImplEx(motorController, 2);
        DcMotorEx motor3 = new DcMotorImplEx(motorController, 3);

        LynxServoController servoController = (LynxServoController) hardwareMap.servoController.iterator().next();
        Servo servo0 = new ServoImpl(servoController, 0);
        Servo servo1 = new ServoImpl(servoController, 1);
        Servo servo2 = new ServoImpl(servoController, 2);
        Servo servo3 = new ServoImpl(servoController, 3);
        Servo servo4 = new ServoImpl(servoController, 4);
        Servo servo5 = new ServoImpl(servoController, 5);

        LynxResetMotorEncoderCommand command = new LynxResetMotorEncoderCommand(PhotonCore.CONTROL_HUB, 2);
        try {
            command.sendReceive();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }
        waitForStart();
        double dir = 0.5;
        double power = 0;
        telemetry.setMsTransmissionInterval(50);
        long last = System.nanoTime();

        double[] loopTimes = new double[50];
        int index = 0;

        while(opModeIsActive()){
            motor2.setPower(power);
            long now = System.nanoTime();
            double dt = (now - last)/1.0e+9;
            last = now;

            power += dir * dt;
            if(power > 0.2){
                power = 0.15;
                dir = -0.25;
            }
            if(power < -0.2){
                power = -0.15;
                dir = 0.25;
            }

            motor0.setPower(Math.random());
            motor1.setPower(Math.random());
            motor2.setPower(Math.random());
            motor3.setPower(Math.random());
            motor2.setPower(Math.random());
            motor3.setPower(Math.random());

            servo0.setPosition(Math.random());
            servo1.setPosition(Math.random());
            servo2.setPosition(Math.random());
            servo3.setPosition(Math.random());
            servo4.setPosition(Math.random());
            servo5.setPosition(Math.random());

            loopTimes[index] = (dt * 1000.0);
            index ++;
            if(index >= loopTimes.length){
                index = 0;
            }

            double avg = 0;
            for(double d : loopTimes){
                avg += d / loopTimes.length;
            }

            if(gamepad1.a){
                enabled = true;
                PhotonCore.enable();
            }
            if(gamepad1.b){
                enabled = false;
                PhotonCore.disable();
            }

            telemetry.addData("Motor Power", power);
            telemetry.addData("Motor Position", motor2.getCurrentPosition());
            telemetry.addData("Interval (ms)", (dt * 1000.0));
            telemetry.addData("Average Interval (ms)", avg);
            telemetry.addData("Run Frequency (hz)", 1.0/dt);

            //telemetry.addData("2M Distance (mm)", vl53L0XEx.getDistance(DistanceUnit.MM));
            //telemetry.addData("Color Sensor V3", revColorSensorV3Ex.getDistance(DistanceUnit.MM));
            //telemetry.addData("MB1242", mb1242Ex.getDistanceAsync(DistanceUnit.MM));
            telemetry.addData("Gyro", imu.getAngularOrientation().firstAngle);

            telemetry.addLine("PhotonCore " + (enabled ? "ENABLED" : "DISABLED"));
            telemetry.update();
        }
    }
}
