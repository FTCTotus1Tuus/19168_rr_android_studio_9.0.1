package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.*;

import java.util.*;
import java.io.*;

@TeleOp

public class IntakeTest extends LinearOpMode{

    CRServo leftIntake;
    CRServo rightIntake;
    CRServo feeder;
    double[] direction = {0.0,0.0};
    double rotation;
    double regularDivBy = 1;
    double turboDivBy = 1;
    boolean turboBoost;

    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right


    public void runOpMode(){
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor3");
        omniMotor2 = initializeMotor("omniMotor1");
        omniMotor3 = initializeMotor("omniMotor2");

        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

        feeder = hardwareMap.get(CRServo.class, "feeder");
        waitForStart();

        //Start
        while (this.opModeIsActive()) {

            feeder.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

        if (gamepad1.y) {
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        }  else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }


            direction[0] = -gamepad1.left_stick_x;
            direction[1] = -gamepad1.left_stick_y;
            rotation = -gamepad1.right_stick_x;
            turboBoost = gamepad1.left_stick_button;


            MoveRobot(direction, -rotation, turboBoost);
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