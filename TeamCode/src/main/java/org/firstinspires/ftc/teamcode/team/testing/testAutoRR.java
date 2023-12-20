package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;


@Autonomous
public class testAutoRR extends DarienOpModeAuto {
        @Override
        public void runOpMode() {
            IMU imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
            imu.initialize(parameters);

            imu.resetYaw();


            leftArm = initializeMotor("leftArm");
            rightArm = initializeMotor("rightArm");

            leftArm.setDirection(DcMotor.Direction.REVERSE);

            if (true) {
                leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
            rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

            clawWrist = hardwareMap.get(Servo.class, "clawWrist");
            clawLeft = hardwareMap.get(Servo.class, "clawLeft");
            clawRight = hardwareMap.get(Servo.class, "clawRight");

            feeder = hardwareMap.get(CRServo.class, "feeder");
            initCamera(false);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)));


//            Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();
//            Trajectory middleTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();
//            Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();

            Trajectory mainTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)))

                    .forward(33)
//                    .addDisplacementMarker(75, () -> {
//                        setArmPosition("out");
//                    })
//                    .addDisplacementMarker(80, () -> {
//                        setWristPosition("drop");
//                    })
                    .splineTo(new Vector2d(-28.47, -12.30), Math.toRadians(6.31))
                    .splineTo(new Vector2d(5.90, -10.26), Math.toRadians(-1.33))
                    .splineTo(new Vector2d(45, -37.06), Math.toRadians(0.00))
                    .build();


            waitForStart();

            if(isStopRequested()) return;
//            switch (teamPropMaskPipeline.getLastResults()) {
//                case 1:
////                    drive.followTrajectory(leftTrajectory);
//                    telemetry.addData("1", "");
//                    break;
//                case 2:
////                    drive.followTrajectory(middleTrajectory);
//
//                    telemetry.addData("2", "");
//                    break;
//                case 3:
////                    drive.followTrajectory(rightTrajectory);
//
//                    telemetry.addData("3", "");
//                    break;
//            }


//                telemetry.addData("",teamPropMaskPipeline.getLastResults());
                telemetry.update();


            drive.followTrajectory(mainTrajectory);
            setClawPosition("open");
        }
    }

