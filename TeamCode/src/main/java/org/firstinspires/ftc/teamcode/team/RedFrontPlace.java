package org.firstinspires.ftc.teamcode.team;

import static android.opengl.Matrix.length;

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

//        propPosition = teamPropMaskPipeline.getLastResults();
        propPosition = 3; // camera is mounted upside down. 3=1 and 1=3.
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(26, 0.3); //centers on spike tile
            sleep(350);
            setArmPosition(150, 0.3); // extends the arm
            sleep(500);
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            case 3:
                AutoRotate(90, 0.3,-1); // turns to spike mark
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveX(24, 0.3);  // moves 1 tile right to be facing the backdrop
                waitForMotors();

                AutoRotate(-90, 0.3, 1);

                autoRunMacro("ReadyToPickup"); // returns the wrist

                break;
            case 2:
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(-24, 0.3); // goes 1 tile towards the pixel piles
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(-90, 0.3, 1); // turns towards backdrop
                MoveX(-24, 0.3); // moves in line with top case
                waitForMotors();
                MoveY(24, 0.3); // centers bot
                waitForMotors();
                break;
            case 1:
                AutoRotate(-90, 0.3,1); // turns to spike mark
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(-24, 0.3); // moves in line with top case
                waitForMotors();
                autoRunMacro("ReadyToPickup"); // returns the wrist
                break;
        }

        MoveY(72,0.3); // moves past stage door towards backdrop
        waitForMotors();
            setClawPosition("closed"); // grabs yellow pixel
        MoveX(20, 0.3);//in front of backdrop
        waitForMotors();
        setArmPosition(250, 0.2); // extends the arm a tiny bit
        autoRunMacro("ReadyToDrop"); // extends the wrist
        print("pls no crash","");
        backDropPlace(false, propPosition);

        //Parking
        MoveX(-12, 0.2);
    }
}
