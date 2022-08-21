package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.outoftheboxrobotics.photoncore.PhotonCore;
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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

@TeleOp
public class SingleHubValidation extends LinearOpMode {
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

        LynxServoController servoController = null;
        try {
            servoController = new LynxServoController(hardwareMap.appContext, PhotonCore.CONTROL_HUB);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        ServoImplEx servo0 = (ServoImplEx) hardwareMap.servo.get("s0");
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

        waitForStart();

        double dir = 0.5;
        double power = 0;
        telemetry.setMsTransmissionInterval(15);
        long last = System.nanoTime();

        double[] loopTimes = new double[50];
        int index = 0;

        boolean lastUp = false, lastDown = false;

        int numCommands = 4;

        double dt3 = 1;

        long now2 = 0;

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

            telemetry.addData("IMU angle", imu.getAngularOrientation().firstAngle);
            long read = imu.getAngularOrientation().acquisitionTime;
            if(read != now2) {
                dt3 = (read - now2) / 1.0E9;
                now2 = read;
            }

            telemetry.addData("Update Rate (HZ)", 1.0/dt3);

            PhotonCore.experimental.setMaximumParallelCommands(numCommands);

            telemetry.addData("Parallel Commands", numCommands);
            telemetry.addLine("PhotonCore " + (enabled ? "ENABLED" : "DISABLED"));

            long timer6 = System.currentTimeMillis();

            telemetry.update();
        }
    }
}
