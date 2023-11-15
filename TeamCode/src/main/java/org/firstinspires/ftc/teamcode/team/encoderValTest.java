package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
@TeleOp
public class encoderValTest extends DarienOpMode {
    public void runOpMode() {
        initControls(true);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                AutoRotate(90, power, -1);
            } else if (gamepad1.b) {
                MoveX(5, power);
            } else if (gamepad1.x) {
                MoveY(5, power);
            }
            else if (gamepad1.y) {
                setArmPosition(250, 0.1);
            }
        }
    }
}
