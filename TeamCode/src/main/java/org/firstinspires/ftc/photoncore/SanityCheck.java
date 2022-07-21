package org.firstinspires.ftc.photoncore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SanityCheck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor0 = hardwareMap.dcMotor.get("0");
        DcMotor motor1 = hardwareMap.dcMotor.get("1");
        DcMotor motor2 = hardwareMap.dcMotor.get("2");
        DcMotor motor3 = hardwareMap.dcMotor.get("3");

        Servo servo0 = hardwareMap.servo.get("s0");
        Servo servo1 = hardwareMap.servo.get("s1");
        Servo servo2 = hardwareMap.servo.get("s2");
        Servo servo3 = hardwareMap.servo.get("s3");
        Servo servo4 = hardwareMap.servo.get("s4");
        Servo servo5 = hardwareMap.servo.get("s5");

        waitForStart();
        telemetry.setMsTransmissionInterval(15);
        long start = System.currentTimeMillis();
        double[] intervals = new double[20];
        int i = 0;
        while (opModeIsActive()){
            long now = System.currentTimeMillis();
            double dt = (now - start);
            intervals[i] = dt;
            i ++;
            if(i >= intervals.length){
                i = 0;
            }
            start = now;
            telemetry.addData("Interval", dt);
            double avg = 0;
            for(double d : intervals){
                avg += d;
            }
            avg = avg / ((double)intervals.length);
            telemetry.addData("Average", avg);
            telemetry.update();
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

        }
    }
}
