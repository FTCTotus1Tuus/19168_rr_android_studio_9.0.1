package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueBackPlace extends DarienOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initCamera(true);
        initControls(true);

        int propPosition;

        waitForStart();

        propPosition = getPropPosition();
        telemetry.addData("Prop", teamPropMaskPipeline.getLastResults());
        telemetry.update();
        autoRunMacro("ReadyToPickup");
        setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
        MoveY(24, 0.3); //centers on spike tile
            sleep(350);
            setArmPosition(250, 0.3); // extends the arm
            sleep(500);
            setWristPosition("dropGround"); // extends the wrist
        waitForMotors();
        switch (propPosition) {
            case 3:
                AutoRotate(-90, 0.3, 1); // turns to spike mark
                autoRunMacro("dropPixel"); // places the purple pixel on the ground
                MoveY(-24, 0.3);  // moves 1 tile back to be facing the backdrop
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(90, 0.3, -1);
                MoveX(5, 0.3);
                waitForMotors();
                break;
            case 2:
                autoRunMacro("dropPixel"); // places the pixel
                MoveX(-24, 0.3); // goes 1 tile towards the pixel piles
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                AutoRotate(90, 0.3, -1);
                break;
            case 1:
                AutoRotate(90, 0.3, -1); // turns to spike mark
                MoveY(0.5, 0.1);
                waitForMotors();
                autoRunMacro("dropPixel"); // places the pixel
                MoveY(-0.5, 0.1);
                autoRunMacro("ReadyToPickup"); // returns the wrist
                waitForMotors();
                MoveX(-24, 0.3); // strafe left: moves in line with top case
                waitForMotors();
                MoveY(24, 0.3);
                waitForMotors();
                MoveX(18, 0.3);
                break;
        }
        autoRunMacro("ReadyToPickup");
        setArmPosition(-10,0.1);
        sleep(500);
            setClawPosition("closed"); // grabs yellow pixel
        sleep(250);
        setArmPosition(250, 0.1); // extends the arm a tiny bit
        while (leftArm.isBusy()) {}
        autoRunMacro("ReadyToDrop"); // extends the wrist
        print("pls no crash", "");
        autoPlacePixel();
        switch (propPosition) {
            case 1:
                MoveX(-15, 0.3);
                waitForMotors();
                break;
            case 2:
                MoveX(-24, 0.3);
                waitForMotors();
                break;
            case 3:
                MoveX(-31, 0.3);
                waitForMotors();
                break;
        }
        MoveY(10, 0.05);
        waitForMotors();
    }

}
