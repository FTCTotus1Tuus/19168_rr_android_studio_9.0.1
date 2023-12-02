package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class colourSensorDebug extends DarienOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initControls(true);

        waitForStart();
        while(opModeIsActive()) {
        telemetry.addData("Blue:" , isOnLine(true));
        telemetry.addData("Red:" , isOnLine(false));

        telemetry.addData("leftSensor blue", colourSensorLeft.blue());
        telemetry.addData("leftsensor red", colourSensorLeft.red());

        telemetry.addData("rightSensor blue", colourSensorRight.blue());
        telemetry.addData("rightsensor red", colourSensorRight.red());

        telemetry.update();
}}}
