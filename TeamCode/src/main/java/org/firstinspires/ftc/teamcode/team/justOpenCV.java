package org.firstinspires.ftc.teamcode.team;


/* import com.qualcomm.robotcore.eventloop.opmode.Disabled; */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp
public class justOpenCV extends LinearOpMode {



    TeamPropMaskPipeline teamPropMaskPipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        teamPropMaskPipeline = new TeamPropMaskPipeline();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("error openCv", errorCode);
            }
        });



        waitForStart();

        camera.setPipeline(teamPropMaskPipeline);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("cound", getRuntime());

//                if (gamepad1.a) {
                telemetry.addData("Partisian", teamPropMaskPipeline.getLastResults());
//                }

                // Push telemetry to the Driver Station.
                telemetry.update();


                // Share the CPU.
                sleep(20);
            }
        }

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */

}   // end class
