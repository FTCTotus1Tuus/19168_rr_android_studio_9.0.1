package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
@Disabled
@TeleOp
public class gyroTesst extends LinearOpMode {

    public void runOpMode() {


    BHI260IMU.Parameters parameters = new IMU.Parameters(

    new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));


    BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("", imu.getRobotYawPitchRollAngles());
            telemetry.update();
        }
}}
