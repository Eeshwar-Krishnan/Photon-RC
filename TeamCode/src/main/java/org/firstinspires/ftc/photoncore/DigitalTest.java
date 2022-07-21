package org.firstinspires.ftc.photoncore;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
public class DigitalTest extends LinearOpMode {
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

        imu.initialize(parameters);
        waitForStart();
        LynxResetMotorEncoderCommand command1 = new LynxResetMotorEncoderCommand(PhotonCore.CONTROL_HUB, 0);
        try {
            command1.sendReceive();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }
        command1 = new LynxResetMotorEncoderCommand(PhotonCore.CONTROL_HUB, 1);
        try {
            command1.sendReceive();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }
        command1 = new LynxResetMotorEncoderCommand(PhotonCore.CONTROL_HUB, 2);
        try {
            command1.sendReceive();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }
        command1 = new LynxResetMotorEncoderCommand(PhotonCore.CONTROL_HUB, 3);
        try {
            command1.sendReceive();
        } catch (LynxNackException e) {
            e.printStackTrace();
        }
        while(opModeIsActive()){
            LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(PhotonCore.CONTROL_HUB);
            LynxGetModuleStatusCommand command3 = new LynxGetModuleStatusCommand(PhotonCore.CONTROL_HUB);
            try {
                LynxGetBulkInputDataResponse response = command.sendReceive();
                LynxGetModuleStatusResponse response1 = command3.sendReceive();
                RobotLog.ee("Encoder", response.getEncoder(0) + " | " + response.getEncoder(1) + " | " + response.getEncoder(2) + " | " + response.getEncoder(3) + " | " + response1.hasMotorLostCounts(0) + " | " + response1.hasMotorLostCounts(1) + " | " + response1.hasMotorLostCounts(2) + " | " + response1.hasMotorLostCounts(3));
            } catch (LynxNackException e) {
                e.printStackTrace();
            }
            for(int i = 0; i < 4; i ++){
                LynxSetMotorConstantPowerCommand command2 = new LynxSetMotorConstantPowerCommand(PhotonCore.CONTROL_HUB, 0, 0);
                try {
                    command2.sendReceive();
                } catch (LynxNackException e) {
                    e.printStackTrace();
                }
            }
            imu.getAngularOrientation();
        }
    }
}
