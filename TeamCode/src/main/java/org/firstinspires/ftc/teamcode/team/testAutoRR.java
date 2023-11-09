package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class testAutoRR extends DarienOpMode{
        @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)));


            Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)));
            Trajectory middleTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)));
            Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)));

            Trajectory mainTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)))

                    .forward(33)
                    .splineTo(new Vector2d(-28.47, -12.30), Math.toRadians(6.31))
                    .splineTo(new Vector2d(5.90, -10.26), Math.toRadians(-1.33))
                    .splineTo(new Vector2d(51.32, -37.06), Math.toRadians(0.00))
                    .build();


            waitForStart();

            switch (teamPropMaskPipeline.getLastResults()) {
                case 1:
                    drive.followTrajectory(leftTrajectory);
                    break;
                case 2:
                    drive.followTrajectory(middleTrajectory);
                    break;
                case 3:
                    drive.followTrajectory(rightTrajectory);
                    break;
            }


            if(isStopRequested()) return;

            drive.followTrajectory(myTrajectory);
        }
    }

