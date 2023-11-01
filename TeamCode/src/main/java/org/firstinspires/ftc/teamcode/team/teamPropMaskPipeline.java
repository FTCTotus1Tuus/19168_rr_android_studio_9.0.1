package org.firstinspires.ftc.teamcode.team;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

@Config
class TeamPropMaskPipeline implements VisionProcessor
{
        public static int maskSel = 3, minHue = 80, minSaturation = 0, minValue = 20 , maxHue = 117, maxSaturation = 300, maxValue = 200, lastResults = 1, frameWidth, frameHeight;
        Rect firstThird, secondThird, thirdThird;
        double avgFirstThird, avgSecondThird, avgThirdThird;
        Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMat3 = new Mat();
        Mat mask1 = new Mat(), mask2 = new Mat(), mask3 = new Mat();

        public static double clawPositionL;
        public static double clawPositionR;




    public int getLastResults() {
        return lastResults;
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

//            workingMat1.release();
//            workingMat2.release();
//            workingMat3.release();
//
//            mask1.release();
//            mask2.release();
//            mask3.release();


        // Calculate the width and height of the frame
        frameWidth = frame.width();
        frameHeight = frame.height();


        // Define the region of interest (ROI) for each third of the screen
        firstThird = new Rect(0, 0, frameWidth / 3, frameHeight);
        secondThird = new Rect(frameWidth / 3, 0, frameWidth / 3, frameHeight);
        thirdThird = new Rect(2 * (frameWidth / 3), 0, frameWidth / 3, frameHeight);

        workingMat1 = frame.clone();


        // Apply a color mask to the workingMat (for example, to highlight a specific color)


        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat2, new Scalar(minHue, minSaturation, minValue), new Scalar(maxHue, maxSaturation, maxValue), workingMat3);

        mask1 = workingMat3.submat(firstThird);
        mask2 = workingMat3.submat(secondThird);
        mask3 = workingMat3.submat(thirdThird);


        avgFirstThird = Core.countNonZero(mask1);
        avgSecondThird = Core.countNonZero(mask2);
        avgThirdThird = Core.countNonZero(mask3);


//             You can now use the average values to determine the object's position
        if (avgFirstThird > avgSecondThird && avgFirstThird > avgThirdThird) {lastResults = 1;}
        else if (avgSecondThird > avgFirstThird && avgSecondThird > avgThirdThird) {lastResults = 2;}
        else { lastResults = 3;}


        workingMat1.release();
        workingMat2.release();
        workingMat3.release();

        mask1.release();
        mask2.release();
        mask3.release();


        return frame;
//            return workingMat3.adjustROI(0,0,frameWidth,frameHeight);
//        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}