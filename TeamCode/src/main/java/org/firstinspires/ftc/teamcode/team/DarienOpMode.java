package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class DarienOpMode extends LinearOpMode {

    public CRServo leftIntake;
    public CRServo rightIntake;
    public CRServo feeder;
    public CRServo droneLauncher;

    public Servo clawWrist;
    public Servo clawLeft;
    public Servo clawRight;

    public ColorSensor colourSensorLeft, colourSensorRight;
    public static int minRedVal = 900, minBlueVal = 1000;

    public static double encoderResolution = 537.7 ; //no change unless we change motors
    public double wheelDiameter = 3.75; // inches

    public double constMult = (wheelDiameter * (Math.PI));
    public double constant = encoderResolution / constMult;
    public static double rotationTolerance = 5;
    public static double power = 0.3;
    public int encoderPos0;
    public int encoderPos1;
    public int encoderPos2;
    public int encoderPos3;
    double clawWristPositionPickup = 0.71;
    double clawWristPositionDrop = 0.3;
    public static double clawWristPositionGround = 0.6;
    // l open is 0.4 closed is 0.1
    // r open is 0.6 closed is 0.9 for claw servos
    public double clawLeftPositionOpen = 0.4;
    public double clawLeftPositionClosed = 0.08;
    public double clawRightPositionOpen = 0.6;
    public double clawRightPositionClosed = 0.93;

    public static int armOutPosition = 750;

    public double[] direction = {0.0, 0.0};
    public double rotation;
    public int encoderPos;
    public double regularDivBy = 1;
    public double turboDivBy = 1;
    public boolean turboBoost;

    public DcMotor omniMotor0; // front left
    public DcMotor omniMotor1; // front right
    public DcMotor omniMotor2; // back left
    public DcMotor omniMotor3; // back right

    public DcMotor leftArm;
    public DcMotor rightArm;

    public IMU imu;

    public VisionPortal visionPortal;
    public TeamPropMaskPipeline teamPropMaskPipeline;
    public AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {}

    public void runDroneSystem(){
        if(gamepad2.left_stick_button && gamepad2.b){
            droneLauncher.setPower(0.5);
            sleep(500);
        } else {
            droneLauncher.setPower(0);
        }
    }
    public void runIntakeSystem() {
        if (gamepad1.right_bumper) {
            // Load pixels
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        } else if (gamepad1.right_trigger > 0) {
            // Eject pixels
            leftIntake.setPower(-1);
            rightIntake.setPower(1);
        } else {
            // Stop
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void runFeederSystem() {
        if (gamepad1.left_bumper) {
            // Load pixels
            feeder.setPower(-1);
        } else if (gamepad1.left_trigger > 0) {
            // Eject pixels
            feeder.setPower(1);
        } else {
            // Stop
            feeder.setPower(0);
        }
    }

    public void runWristSystem() {
        // Wrist to be on fixed positions for grabbing and for dropping on the backboard.
        if (gamepad2.right_trigger > 0) {
            // Pick up pixels from the robot tray.
            setWristPosition("pickup");
        }
        if (gamepad2.left_trigger > 0) {
            // Drop pixels on backboard
            setWristPosition("drop");
        }
        if (gamepad2.x) {
            setWristPosition("dropGround");
        }
        //clawWrist.setPower(gamepad2.left_stick_y/3); // clawWrist control as a continuous servo (CRServo).
    }

    public void setWristPosition(String position) {
        switch (position) {
            case "pickup":
                clawWrist.setPosition(clawWristPositionPickup);
                break;
            case "drop":
                clawWrist.setPosition(clawWristPositionDrop);
                break;
            case "dropGround":
                clawWrist.setPosition(clawWristPositionGround);
                break;
            default:
                // do nothing;
        }
    }

    public void runClawSystem() {
        if (gamepad2.right_bumper) {
            setClawPosition("open");
            // Open the claw
        } else if (gamepad2.left_bumper) {
            setClawPosition("closed");
            // Close the claw
        }

    }

    public void setClawPosition(String position) {
        switch (position) {
            case "open":
                clawLeft.setPosition(clawLeftPositionOpen);
                clawRight.setPosition(clawRightPositionOpen);
                break;
            case "closed":
                clawLeft.setPosition(clawLeftPositionClosed);
                clawRight.setPosition(clawRightPositionClosed);
                break;
            case "leftOpen":
                clawLeft.setPosition(clawLeftPositionOpen);
                break;
            case "rightOpen":
                clawRight.setPosition(clawRightPositionOpen);
                break;
            case "leftClosed":
                clawLeft.setPosition(clawLeftPositionClosed);
                break;
            case "rightClosed":
                clawRight.setPosition(clawRightPositionClosed);
                break;
            default:
                // do nothing;
        }
    }

    public void runArmSystem() {
        // DONE: Arm control on the left joystick up/down.
        leftArm.setPower(-gamepad2.left_stick_y);
        rightArm.setPower(-gamepad2.left_stick_y);
        //leftArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
        //rightArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
    }

    public void setArmPosition(int position, double power) {
        //positive is out
        rightArm.setTargetPosition(position);
        leftArm.setTargetPosition(position);

        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightArm.setPower(power);
        leftArm.setPower(power);
    }

    public void driveArm(String position) {
        double power = 0.8;
        switch (position) {
            case "in":
                // TODO: power the arm down until it reaches the stopping position. Then turn off the power.
                leftArm.setPower(-power);
                rightArm.setPower(-power);
                break;
            case "out":
                leftArm.setPower(power);
                rightArm.setPower(power);
                break;
            case "none":
                leftArm.setPower(0);
                rightArm.setPower(0);
            default:
                // do nothing;
        }
    }

    public void runDriveSystem() {
        direction[0] = (Math.pow(-gamepad1.left_stick_x, 7) -gamepad1.left_stick_x)/2;
        direction[1] = (Math.pow(-gamepad1.left_stick_y, 7) -gamepad1.left_stick_y)/2;
        rotation = (Math.pow(-gamepad1.right_stick_x, 7) -gamepad1.right_stick_x) / 2;
        turboBoost = gamepad1.left_stick_button;

        MoveRobot(direction, -rotation, turboBoost);
    }

    public void runMacro(String macro) {
        switch (macro) {
            case "ReadyToPickup":
                if (gamepad2.y) {
                    if (clawWrist.getPosition() < clawWristPositionPickup) {
                        // If the wrist is already in PICKUP position, do nothing.
                        setWristPosition("pickup");
                    }
                    if (clawLeft.getPosition() < clawLeftPositionOpen) {
                        // If the claw is already in OPEN position, do nothing.
                        setClawPosition("open");
                    }
                }
                while (gamepad2.y) {
                    driveArm("in");
                }
                break;
            case "ReadyToDrop":
                if (gamepad2.a) {
                    if (clawWrist.getPosition() > clawWristPositionDrop) {
                        // Only raise the wrist if it's not already in position.
                        setWristPosition("drop");
                    }
                }
                while (gamepad2.a) {
                    driveArm("out");
                }
                break;
            default:
                // do nothing;
                break;
        }
    }
    public void autoRunMacro(String macro) {
        switch (macro) {
            case "ReadyToPickup":
                setWristPosition("pickup");
                setClawPosition("open");

                sleep(50);

                leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                driveArm("in");
                sleep(200);
                driveArm("none");

                leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case "dropPixel":
                //always put purple pixel to the left
                setClawPosition("open");
                sleep(1000);
                break;
            case "ReadyToDrop":
                setWristPosition("drop");
                break;
            default:
                // do nothing;
                break;
        }
    }

    public void MoveRobot(double[] direction, double rotation, boolean turboBoost) {

        double divBy;
        double wheel0 = clamp(-direction[0] + direction[1] + rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] - rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] - rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] + rotation, -1, 1);
        if (turboBoost) {
            divBy = turboDivBy;
        } else {
            divBy = regularDivBy;

        }
        telemetry.addData("", wheel0 / divBy);

        MoveMotor(omniMotor0, wheel0 / divBy);
        MoveMotor(omniMotor1, wheel1 / divBy);
        MoveMotor(omniMotor2, wheel2 / divBy);
        MoveMotor(omniMotor3, wheel3 / divBy);
    }


    public void initCamera(boolean isBlue) {
        // true = blue false = red
        teamPropMaskPipeline = new TeamPropMaskPipeline(isBlue);
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(teamPropMaskPipeline);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    public void initControls(boolean isAuto) {
        //isAuto: true=auto false=teleop
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.resetYaw();

        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor3");
        omniMotor2 = initializeMotor("omniMotor1");
        omniMotor3 = initializeMotor("omniMotor2");

        leftArm = initializeMotor("leftArm");
        rightArm = initializeMotor("rightArm");

        leftArm.setDirection(DcMotor.Direction.REVERSE);

        if (isAuto) {
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            colourSensorLeft = hardwareMap.get(ColorSensor.class, "colourSensorLeft");
            colourSensorRight = hardwareMap.get(ColorSensor.class, "colourSensorRight");
        }


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
        droneLauncher = hardwareMap.get(CRServo.class, "droneLauncher");
    }

    public boolean isOnLine(boolean isBlue) {
        if (isBlue && (colourSensorLeft.blue() > minBlueVal || colourSensorRight.blue() > minBlueVal)) {
            return true;
        } else if (!isBlue && (colourSensorRight.red() > minRedVal || colourSensorLeft.red() > minRedVal)) {
            return true;
        } else {return false;}
    }
    public void backDropPlace(boolean isBlue, int propPosition) {
        if (isBlue) {
            switch (propPosition) {
                case 3:
                    MoveX(-22, 0.3);
                    waitForMotors();
                    break;
                case 2:
                    MoveX(-28.5, 0.3);
                    waitForMotors();
                    break;
                case 1:
                    MoveX(-32, 0.3);
                    waitForMotors();
                    break;
            } AutoRotate(90F, 0.3, 0);
        }
        else {
            // RED: STRAFE RIGHT TO THE CORRECT POSITION BASED ON THE STRIPE MARK
            switch (propPosition) {
                case 1:
                    MoveX(16, 0.3);
                    waitForMotors();
                    break;
                case 2:
                    MoveX(26, 0.3);
                    waitForMotors();
                    break;
                case 3:
                    MoveX(32, 0.3);
                    waitForMotors();
                    break;
            }
            AutoRotate(-90, 0.3, 0);
        }
            while (!isOnLine(isBlue)) { MoveY(1, 0.2);}
            MoveY(3.5,0.1); // chagne to 2.5
            waitForMotors();
        autoPlacePixel();
    }

    public void autoPlacePixel() {
        waitForMotors();
        autoRunMacro("dropPixel");
        MoveY(-3, 0.25);
        waitForMotors();
        autoRunMacro("ReadyToPickup");
        return;
    }

    public void MoveY(double y, double power) {

        resetEncoder();
        encoderPos = (int) Math.floor((y * constant + 0.5));
        setTargetPosY();
        print("moving y",encoderPos);
        setRunMode();

        setPower(power);
    }


    public void MoveX(double x, double power) {
        resetEncoder();

        encoderPos = (int) Math.floor((x * constant * 1.111111) + 0.5);
        print("moving x",encoderPos);
        setTargetPosX();
        setRunMode();
        setPower(power);
    }

    public void AutoRotate(double TargetPosDegrees, double power, int direction) {
        //direction counter clockwise is -1 clockwise is 1
        setToRotateRunMode();
        double initError = Math.abs(TargetPosDegrees-getRawHeading());
        double truePower;
        double error;
        double scaleConstant = initError;
        boolean isRotating = true;

        if (direction == 0) {
            if ((TargetPosDegrees-getRawHeading()) > 0) {direction=-1;}
            else {direction = 1;}
            rotationTolerance = 2;
        }

        while (isRotating) {
            error = Math.abs(TargetPosDegrees-getRawHeading());
//            truePower = Math.min(Math.pow((error/scaleConstant),2),1);
            telemetry.addData("heading ", getRawHeading());
            telemetry.addData("error ", error);
            print("", power);

            setRotatePower(power, direction);
            if (error<rotationTolerance) {
                isRotating = false;
                setRotatePower(0,0);
                resetEncoder();
                return;

            }

        }
    }
    public int getPropPosition() {
        MoveY(2, 0.3);
        waitForMotors();
        AutoRotate(10, 0.3, -1);
        double firstResults = teamPropMaskPipeline.getLastResults()[0];
        double secondResults1 = teamPropMaskPipeline.getLastResults()[1];
        AutoRotate(-10, 0.3, 1);
        double thirdResults = teamPropMaskPipeline.getLastResults()[2];
        double secondResults2 = teamPropMaskPipeline.getLastResults()[1];

        double secondResults = (secondResults1+secondResults2)/2;

        AutoRotate(0, 0.3, -1);

        if (firstResults > secondResults && firstResults > thirdResults) {return 1;}
        else if (secondResults > firstResults && secondResults > thirdResults) {return 2;}
        else {return 3;}

    }

        public double sigmoid(double x) {
        //takes in any x value returns from (0,0) to (1,1) scale x accordingly
        return (2/(1+Math.pow(2.71,(-4*x))))-1;
        }
        public double getVoltage () {
            return (hardwareMap.voltageSensor.iterator().next().getVoltage());
        }


        public void setTargetPosY ()
        {

            omniMotor0.setTargetPosition(encoderPos);
            omniMotor1.setTargetPosition(encoderPos);
            omniMotor2.setTargetPosition(-encoderPos);
            omniMotor3.setTargetPosition(-encoderPos);

        }

        public void setTargetPosX ()
        {

            omniMotor0.setTargetPosition(encoderPos);
            omniMotor1.setTargetPosition(-encoderPos);
            omniMotor2.setTargetPosition(encoderPos);
            omniMotor3.setTargetPosition(-encoderPos);

        }



        public void setRunMode ()
        {
            omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void setToRotateRunMode () {
            omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        public void setPower ( double power)
        {
            omniMotor0.setPower(relativePower(power));
            omniMotor1.setPower(relativePower(power));
            omniMotor2.setPower(relativePower(power));
            omniMotor3.setPower(relativePower(power));

        }

        public void setRotatePower ( double power, double direction){
            omniMotor0.setPower(relativePower(direction * power));
            omniMotor1.setPower(relativePower(-direction * power));
            omniMotor2.setPower(relativePower(-direction * power));
            omniMotor3.setPower(relativePower(direction * power));

        }

        public void resetEncoder ()
        {
            omniMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            omniMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            omniMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            omniMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void waitForMotors ()
        {
            while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy()) {
            }
        }

        int move_to_position;
        double y;

        public void print (String Name, Object message)
        {
            telemetry.addData(Name, message);
            telemetry.update();
        }
        public double relativePower ( double intended_power)
        {
            return (13 * intended_power) / getVoltage();
        }


        public double getRawHeading () {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        public DcMotor initializeMotor (String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return motor;
        }

        public void MoveMotor (DcMotor motor,double power){
         /*This function just moves the motors and updates the
         logs for replay*/
            motor.setPower(power);
        }
        public static double clamp ( double val, double min, double max){
            return Math.max(min, Math.min(max, val));
        }
    }
