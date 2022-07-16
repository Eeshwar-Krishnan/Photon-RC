package org.firstinspires.ftc.photoncore;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

@TeleOp
public class ServoTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        LynxServoController servoController = (LynxServoController) hardwareMap.servoController.iterator().next();
        Servo servo0 = new ServoImplEx(servoController, 0, ServoConfigurationType.getStandardServoType());
        Servo servo1 = new ServoImplEx(servoController, 1, ServoConfigurationType.getStandardServoType());

        servo0.setPosition(1);

        waitForStart();
        ((ServoImplEx)servo0).setPwmDisable();
        servo1.setPosition(1);
        while (opModeIsActive());
    }
}