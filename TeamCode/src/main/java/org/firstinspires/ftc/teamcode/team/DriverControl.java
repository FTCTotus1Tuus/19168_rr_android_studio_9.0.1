package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class DriverControl extends DarienOpMode{


    public void runOpMode(){

        initControls();

        waitForStart();

        //Start
        while (this.opModeIsActive()) {
            runIntakeSystem();
            runFeederSystem();
            runWristSystem();
            runClawSystem();
            runArmSystem();
            runDriveSystem();

            runMacro("ReadyToPickup");
            runMacro("ReadyToDrop");
        }
    }
}