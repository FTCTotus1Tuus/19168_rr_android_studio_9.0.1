package org.firstinspires.ftc.teamcode.team;

import static android.opengl.Matrix.length;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class RedFrontPlace extends DarienOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(false);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = teamPropMaskPipeline.getLastResults();
        telemetry.addData("Prop", teamPropMaskPipeline.getLastResults());
        telemetry.update();
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(26, 0.3); //centers on spike tile
            sleep(350);
            setArmPosition(250, 0.3); // extends the arm
            sleep(500);
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            case 1:
                AutoRotate(90, 0.3,-1); // turns to spike mark
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveX(24, 0.3);  // moves 1 tile right to be facing the backdrop
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(-90, 0.3, 1);
                break;
            case 2:
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(-20, 0.3); // goes 1 tile towards the pixel piles
                    autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(0, 0.1 ,1);
                MoveY(24, 0.3);
                waitForMotors();
                AutoRotate(-90, 0.3, 1); // turns towards backdrop
                MoveY(20, 0.3); // moves in line with top case
                waitForMotors();
                break;
            case 3:
                AutoRotate(-90, 0.3,1); // turns to spike mark
                MoveY(1.5, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
                MoveY(-1.5, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(-24, 0.3); // moves in line with top case
                waitForMotors();
                break;
        }
        autoRunMacro("ReadyToPickup");
        setArmPosition(-10,0.1);
        MoveY(72,0.3); // moves past stage door towards backdrop
        waitForMotors();
            setClawPosition("closed"); // grabs yellow pixel
        MoveX(20, 0.3);//in front of backdrop
        sleep(200);
        setArmPosition(250, 0.2); // extends the arm a tiny bit
        waitForMotors();
        autoRunMacro("ReadyToDrop"); // extends the wrist
        print("pls no crash","");
        backDropPlace(false, propPosition);

        sleep(250);
        //Parking
        MoveX(-24, 0.2);
        waitForMotors();
        MoveY(10, 0.05);
        waitForMotors();
    }
}