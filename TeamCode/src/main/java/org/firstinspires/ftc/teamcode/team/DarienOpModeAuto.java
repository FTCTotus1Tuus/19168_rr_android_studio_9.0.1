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
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class DarienOpModeAuto extends DarienOpMode {

    public VisionPortal visionPortal;
    public TeamPropMaskPipeline teamPropMaskPipeline;
    public AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {}
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
        //|| colourSensorRight.blue() > minBlueVal
        if (isBlue && (colourSensorLeft.blue() > minBlueVal )) {
            return true;
        } else if (!isBlue && (colourSensorLeft.red() > minRedVal)) {
            return true;
        } else {return false;}
    }
    public void backDropPlace(boolean isBlue, int propPosition) {
        if (isBlue) {
            switch (propPosition) {
                case 3:
                    MoveX(-23, 0.3);
                    waitForMotors();
                    break;
                case 2:
                    MoveX(-26, 0.3);
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
                    MoveX(14.5, 0.3);
                    waitForMotors();
                    break;
                case 2:
                    MoveX(25, 0.3);
                    waitForMotors();
                    break;
                case 3:
                    MoveX(30, 0.3);
                    waitForMotors();
                    break;
            }
            AutoRotate(-90, 0.3, 0);
        }
        moveToBackdrop(isBlue);
        autoPlacePixel();
    }

    public void moveToBackdrop(boolean isBlue) {
        MoveY(24, 0.2);
        while (!isOnLine(isBlue)) {}
        print("found line", "");
        MoveY(3,0.1); // chagne to 2.5
        waitForMotors();
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
            power/=2;
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
        public double getRawHeading () {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }


    }