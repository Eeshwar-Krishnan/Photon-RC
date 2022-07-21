package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
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
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

@TeleOp
public class Validation extends LinearOpMode {
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

        //imu.initialize(parameters);

        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor");

        RevColorSensorV3 distanceSensor2 = hardwareMap.get(RevColorSensorV3.class, "color");

        //MB1242Ex mb1242Ex = hardwareMap.get(MB1242Ex.class, "sonic");

        boolean enabled = false;

        LynxDcMotorController motorController = null;
        try {
            motorController = new LynxDcMotorController(hardwareMap.appContext, PhotonCore.CONTROL_HUB);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        DcMotorEx motor0 = (DcMotorEx) hardwareMap.dcMotor.get("0");
        DcMotorEx motor1 = (DcMotorEx) hardwareMap.dcMotor.get("1");
        DcMotorEx motor2 = (DcMotorEx) hardwareMap.dcMotor.get("2");
        DcMotorEx motor3 = (DcMotorEx) hardwareMap.dcMotor.get("3");

        LynxDcMotorController motorController1 = null;
        try {
             motorController1 = new LynxDcMotorController(hardwareMap.appContext, PhotonCore.EXPANSION_HUB);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        DcMotorEx motor4 = (DcMotorEx) hardwareMap.dcMotor.get("4");

        LynxServoController servoController = null;
        try {
            servoController = new LynxServoController(hardwareMap.appContext, PhotonCore.CONTROL_HUB);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        Servo servo0 = hardwareMap.servo.get("s0");
        Servo servo1 = hardwareMap.servo.get("s1");
        Servo servo2 = hardwareMap.servo.get("s2");
        Servo servo3 = hardwareMap.servo.get("s3");
        Servo servo4 = hardwareMap.servo.get("s4");
        Servo servo5 = hardwareMap.servo.get("s5");

        LynxResetMotorEncoderCommand command = new LynxResetMotorEncoderCommand(PhotonCore.CONTROL_HUB, 2);
        try {
            command.sendReceive();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }

        while (!isStarted()){
            telemetry.addData("2M Validation", distanceSensor instanceof Rev2mDistanceSensorEx);
            telemetry.addData("Colour Sensor Validation", distanceSensor2 instanceof RevColorSensorV3Ex);
            telemetry.update();
        }

        double dir = 0.5;
        double power = 0;
        telemetry.setMsTransmissionInterval(50);
        long last = System.nanoTime();

        double[] loopTimes = new double[50];
        int index = 0;

        AtomicInteger angle = new AtomicInteger(0);
        AtomicLong gyrointerval = new AtomicLong(0);
        final long[] lastGyro = {System.currentTimeMillis()};
        final long[] gyroIntervals = new long[20];
        AtomicInteger gyroIndex = new AtomicInteger(0);
        new Thread(() -> {
            while (opModeIsActive()){
                long now = System.currentTimeMillis();
                gyroIntervals[gyroIndex.getAndIncrement()] = (now - lastGyro[0]);

                if(gyroIndex.get() >= gyroIntervals.length){
                    gyroIndex.set(0);
                }

                double avg = 0;
                for(long l : gyroIntervals){
                    avg += l;
                }

                gyrointerval.set((long) (avg / ((double)gyroIntervals.length)));

                lastGyro[0] = now;
                //angle.set((int) imu.getAngularOrientation().firstAngle);
            }
        });

        boolean lastUp = false, lastDown = false;

        int numCommands = 4;

        while(opModeIsActive()){
            long start = System.currentTimeMillis();
            //motor2.setPower(power);
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

            long timer1 = System.currentTimeMillis();

            motor0.setPower(Math.random());
            motor1.setPower(Math.random());
            motor2.setPower(Math.random());
            motor3.setPower(Math.random());
            motor4.setPower(Math.random());

            servo0.setPosition(Math.random());
            servo1.setPosition(Math.random());
            servo2.setPosition(Math.random());
            servo3.setPosition(Math.random());
            servo4.setPosition(Math.random());
            servo5.setPosition(Math.random());

            long timer2 = System.currentTimeMillis();

            loopTimes[index] = (dt * 1000.0);
            index ++;
            if(index >= loopTimes.length){
                index = 0;
            }

            double avg = 0;
            for(double d : loopTimes){
                avg += d / loopTimes.length;
            }

            long timer3 = System.currentTimeMillis();

            if(gamepad1.a){
                enabled = true;
                PhotonCore.enable();
            }
            if(gamepad1.b){
                enabled = false;
                PhotonCore.disable();
            }
            if(gamepad1.x){
                PhotonCore.experimental.setSinglethreadedOptimized(true);
            }
            if(gamepad1.y){
                PhotonCore.experimental.setSinglethreadedOptimized(false);
            }

            if(gamepad1.dpad_up && !lastUp){
                numCommands ++;
            }
            if(gamepad1.dpad_down && !lastDown){
                numCommands --;
            }

            long timer4 = System.currentTimeMillis();

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            long timer5 = System.currentTimeMillis();

            telemetry.addData("Motor Power", power);
            //telemetry.addData("Motor Position", motor2.getCurrentPosition());
            telemetry.addData("Interval (ms)", (dt * 1000.0));
            telemetry.addData("Average Interval (ms)", avg);
            telemetry.addData("Run Frequency (hz)", 1.0/dt);

            //telemetry.addData("2M Distance (mm)", distanceSensor.getDistance(DistanceUnit.MM));
            //telemetry.addData("Color Sensor V3", revColorSensorV3Ex.getDistance(DistanceUnit.MM));
            //telemetry.addData("MB1242", mb1242Ex.getDistanceAsync(DistanceUnit.MM));
            //angle.set((int) imu.getAngularOrientation().firstAngle);
            telemetry.addData("Gyro", angle.get());
            //telemetry.addData("Gyro Interval (ms)", gyrointerval.get());

            PhotonCore.experimental.setMaximumParallelCommands(numCommands);

            telemetry.addData("Parallel Commands", numCommands);
            telemetry.addLine("PhotonCore " + (enabled ? "ENABLED" : "DISABLED"));

            long timer6 = System.currentTimeMillis();

            //RobotLog.ii("Validation", (timer1 - start) + " | " + (timer2 - timer1) + " | " + (timer3 - timer2) + " | " + (timer4 - timer3) + " | " + (timer5 - timer4) + " | " + (timer6 - timer5));
            telemetry.update();
        }
    }
}
