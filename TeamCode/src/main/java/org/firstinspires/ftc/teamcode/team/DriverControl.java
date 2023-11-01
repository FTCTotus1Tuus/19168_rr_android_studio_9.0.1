package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class DriverControl extends LinearOpMode{

    CRServo leftIntake;
    CRServo rightIntake;
    CRServo feeder;

    Servo clawWrist;
    Servo clawLeft;
    Servo clawRight;
    // l open is 0.4 closed is 0.1
    // r open is 0.6 closed is 0.9 for claw servos
    double clawLeftPositionOpen=0.4;
    double clawLeftPositionClosed=0.08;
    double clawRightPositionOpen=0.6;
    double clawRightPositionClosed=0.93;

    double[] direction = {0.0,0.0};
    double rotation;
    double regularDivBy = 1;
    double turboDivBy = 1;
    boolean turboBoost;

    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right

    DcMotor leftArm;
    DcMotor rightArm;

    public void runOpMode(){
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor3");
        omniMotor2 = initializeMotor("omniMotor1");
        omniMotor3 = initializeMotor("omniMotor2");

        leftArm = initializeMotor("leftArm");
        rightArm = initializeMotor("rightArm");

        leftArm.setDirection(DcMotor.Direction.REVERSE);

        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        feeder = hardwareMap.get(CRServo.class, "feeder");
        waitForStart();

        //Start
        while (this.opModeIsActive()) {
            runIntakeSystem();
            runFeederSystem();
            runWristSystem();
            runClawSystem();
            runArmSystem();
            runDriveSystem();

            if (gamepad2.y) {
                runMacro("ReadyToPickup");
            }
        }
    }

    public void runIntakeSystem(){
        if (gamepad1.right_bumper) {
            // Load pixels
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        } else {
            // Stop
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        if(gamepad1.right_trigger>0) {
            // Eject pixels
            leftIntake.setPower(-1);
            rightIntake.setPower(1);
        } else {
            // Stop
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }
    public void runFeederSystem(){
        if (gamepad1.left_bumper) {
            // Load pixels
            feeder.setPower(-1);
        } else {
            // Stop
            feeder.setPower(0);
        }
        if (gamepad1.left_trigger>0){
            // Eject pixels
            feeder.setPower(1);
        } else {
            // Stop
            feeder.setPower(0);
        }
    }
    public void runWristSystem(){
        // Wrist to be on fixed positions for grabbing and for dropping on the backboard.
        if(gamepad2.right_trigger>0) {
            // Pick up pixels from the robot tray.
            setWristPosition("pickup");
        } else {
            // do nothing
        }
        if (gamepad2.left_trigger>0){
            // Drop pixels on backboard
            setWristPosition("drop");
        } else {
            // do nothing
        }
        //clawWrist.setPower(gamepad2.left_stick_y/3); // clawWrist control as a continuous servo (CRServo).
    }
    public void setWristPosition( String position){
        switch( position ){
            case "pickup":
                clawWrist.setPosition(0.7);
                break;
            case "drop":
                clawWrist.setPosition(0.3);
                break;
            default:
                // do nothing;
        }
    }

    public void runClawSystem(){
        if(gamepad2.right_bumper) {
            setClawPosition("open");
            // Open the claw
        } else if (gamepad2.left_bumper){
            setClawPosition("closed");
            // Close the claw
        }

    }
    public void setClawPosition( String position){
        switch( position ){
            case "open":
                clawLeft.setPosition(clawLeftPositionOpen);
                clawRight.setPosition(clawRightPositionOpen);
                break;
            case "closed":
                clawLeft.setPosition(clawLeftPositionClosed);
                clawRight.setPosition(clawRightPositionClosed);
                break;
            default:
                // do nothing;
        }
    }

    public void runArmSystem(){
        // DONE: Arm control on the left joystick up/down.
        leftArm.setPower(-gamepad2.left_stick_y);
        rightArm.setPower(-gamepad2.left_stick_y);
        //leftArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
        //rightArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
    }
    public void setArmPosition( String position){
        switch( position ){
            case "in":
                // TODO: power the arm down until it reaches the stopping position. Then turn off the power.
                leftArm.setPower(-1);
                rightArm.setPower(-1);
                break;
            case "out":
                break;
            default:
                // do nothing;
        }
    }

    public void runDriveSystem(){
        direction[0] = -gamepad1.left_stick_x;
        direction[1] = -gamepad1.left_stick_y;
        rotation = -gamepad1.right_stick_x;
        turboBoost = gamepad1.left_stick_button;

        MoveRobot(direction, -rotation, turboBoost);
    }
    public void runMacro( String macro ){
        switch( macro ) {
            case "ReadyToPickup":
                setWristPosition("pickup");
                setClawPosition("open");
                setArmPosition("in");
                break;
            default:
                // do nothing;
        }
    }

    public void MoveRobot(double[] direction, double rotation, boolean turboBoost){

        double divBy;
        double wheel0 = clamp(-direction[0] + direction[1] + rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] - rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] - rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] + rotation, -1, 1);
        if (turboBoost) {
            divBy = turboDivBy;
        }
        else {
            divBy = regularDivBy;

        }
        telemetry.addData("", wheel0/divBy);

        MoveMotor(omniMotor0,wheel0/divBy);
        MoveMotor(omniMotor1,wheel1/divBy);
        MoveMotor(omniMotor2,wheel2/divBy);
        MoveMotor(omniMotor3,wheel3/divBy);
    }

    public DcMotor initializeMotor(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void MoveMotor(DcMotor motor, double power){
         /*This function just moves the motors and updates the
         logs for replay*/
        motor.setPower(power);
    }
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}