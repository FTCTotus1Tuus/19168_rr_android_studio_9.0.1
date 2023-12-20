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
           initControlsRR(true);
            initCamera(false);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//            drive.setPoseEstimate(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)));



//            Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();
//            Trajectory middleTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();
//            Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();

            Trajectory centerOnTile = drive.trajectoryBuilder(new Pose2d())
                    .forward(24)
                    .addDisplacementMarker(10, () -> {
                        setArmPosition(250, 0.3);
                        setWristPosition("dropGround");
                    })
                    .build();
            Trajectory propPositionOne = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(24).build(); //25 TODO fix this traj prob messing up pos 1
            Trajectory propPositionTwo = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(24).build();  //20
            Trajectory forwardSixteen = drive.trajectoryBuilder(new Pose2d())
                            .forward(16).build();
            waitForStart();


            if(isStopRequested()) return;

            int propPosition = getPropPositionRR(drive);

            autoRunMacro("ReadyToPickup");
            setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up

            drive.followTrajectory(centerOnTile); //centers on spike tile

            switch (propPosition) {
                case 1:
                    drive.turn(Math.toRadians(90)); // turns to spike mark
                    autoRunMacro("dropPixel"); // places the purple pixel on the ground
                    drive.followTrajectory(propPositionOne);  // moves 1 tile right to be facing the backdrop
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                    drive.turn(180);
                    break;
                case 2:
                    autoRunMacro("dropPixel"); // places the pixel
                    drive.followTrajectory(propPositionTwo); // strafe left: goes 1 tile towards the pixel piles
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                    drive.followTrajectory(centerOnTile);
                   drive.turn(Math.toRadians(-90)); // turns towards backdrop
                   drive.followTrajectory(forwardSixteen);
                    break;
                case 3:
                    drive.turn(Math.toRadians(-90)); // turns to spike mark
                    autoRunMacro("dropPixel"); // places the pixel
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                   drive.followTrajectory(propPositionTwo); // strafe left to the center of the tile, facing the backdrop
                    break;
            }
//            drive.followTrajectory(mainTrajectory);

        }
    }

