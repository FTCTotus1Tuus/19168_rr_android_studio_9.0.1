package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class RedFrontPlace extends DarienOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(true);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = teamPropMaskPipeline.getLastResults();
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed");
        MoveY(26, 0.3); //centers on spike tile
        sleep(350);
        setArmPosition(250, 0.3);
        sleep(500);
        setWristPosition("dropGround");
        waitForMotors();
        switch (propPosition) {
            case 1:
                AutoRotate(90, 0.3,-1);
                autoRunMacro("dropAtTeamProp");
                MoveX(24, 0.3);
                waitForMotors();

                // autoRunMacro("ReadyToPickup");
                // macro not working now
                clawWrist.setPosition(clawWristPositionPickup);
                clawLeft.setPosition(clawLeftPositionOpen);
                clawRight.setPosition(clawRightPositionOpen);
                setArmPosition(50,0.1);
                waitForMotors();
                AutoRotate(-85,0.4, 1);
                break;
            case 2:
                autoRunMacro("dropAtTeamProp");
                MoveX(-12, 0.3);

                setWristPosition("pickup");
                setClawPosition("open");
                setArmPosition(50,0.1);

                waitForMotors();
                AutoRotate(-90, 0.3, 1);
                MoveX(-12, 0.3);
                waitForMotors();
                MoveY(12, 0.3);
                waitForMotors();
                break;
            case 3:
                AutoRotate(-90, 0.3,1);
                autoRunMacro("dropAtTeamProp");
                MoveX(-12, 0.3);
                waitForMotors();

                setWristPosition("pickup");
                setClawPosition("open");
                setArmPosition(50,0.1);
                break;
        }

        MoveY(72,0.3);
        waitForMotors();
        MoveX(24, 0.3);//in front of backdrop
        waitForMotors();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null && detection.id == ) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//
//            }
//        }   // end for() loop
//        telemetry.update();
    }
}
