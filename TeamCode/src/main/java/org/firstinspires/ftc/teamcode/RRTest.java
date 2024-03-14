package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.TimeUnit;


@Autonomous
public class RRTest extends LinearOpMode {
    public static class BLUEIDENTIFICATION implements VisionProcessor {
        Mat mixture_LEFT = new Mat();
        Mat mixture_MIDDLE = new Mat();
        Mat mixture_RIGHT = new Mat();
        private boolean blueDetected = false;
        private boolean propLeft = false;
        private boolean propMiddle = false;
        private boolean propRight = false;

        static final Rect ROI_LEFT = new Rect(
                new Point(0, 175),
                new Point(150, 325));

        static final Rect ROI_MIDDLE = new Rect(
                new Point(200, 100),
                new Point(450, 375));

        static final Rect ROI_RIGHT = new Rect(
                new Point(500, 175),
                new Point(640, 325));

        final double PERCENT_THRESHOLD = 0.4;
        final double PERCENT_THRESHOLD_PROP = 0.5;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            //don't need this
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            Imgproc.cvtColor(input, mixture_LEFT, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, mixture_MIDDLE, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, mixture_RIGHT, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(92, 50, 60);
            Scalar highHSV = new Scalar(160, 250, 250);
            Core.inRange(mixture_LEFT, lowHSV, highHSV, mixture_LEFT);
            Core.inRange(mixture_MIDDLE, lowHSV, highHSV, mixture_MIDDLE);
            Core.inRange(mixture_RIGHT, lowHSV, highHSV, mixture_RIGHT);

            Mat rectangleLeft = mixture_LEFT.submat(ROI_LEFT);
            Mat rectangleMiddle = mixture_MIDDLE.submat(ROI_MIDDLE);
            Mat rectangleRight = mixture_RIGHT.submat(ROI_RIGHT);
            double rectanglePercentageLeft = Core.sumElems(rectangleLeft).val[0] / ROI_LEFT.area() / 255;
            double rectanglePercentageMiddle = Core.sumElems(rectangleLeft).val[0] / ROI_MIDDLE.area() / 255;
            double rectanglePercentageRight = Core.sumElems(rectangleLeft).val[0] / ROI_RIGHT.area() / 255;
            rectangleLeft.release();
            rectangleMiddle.release();
            rectangleRight.release();

            if (rectanglePercentageLeft > PERCENT_THRESHOLD) {
                propLeft = true;
            }
            else if (rectanglePercentageMiddle > PERCENT_THRESHOLD_PROP) {
                propMiddle = true;
            }
            else if (rectanglePercentageRight > PERCENT_THRESHOLD) {
                propRight = true;
            }
            return null;

        }

        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            //nothing here
        }
    }//end process

    BLUEIDENTIFICATION blueIdentificationProcess;
    VisionPortal visionPortal;
    static final double Y_SHIFT = 6;
    double coordinateX = 0;
    double coordinateY = 0;
    double heading = 0;
    private DcMotor armBase;
    private Servo wrist;
    private DcMotor armExtension;
    private Servo claw;
    private Servo winchLock;

    private AprilTagProcessor aprilTag;
    // Used for managing the AprilTag detection process.




    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d initPosition = new Pose2d(-48, 0, Math.toRadians(0));
        drive.setPoseEstimate(initPosition);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(initPosition)
                .splineTo(new Vector2d(-19.19, -0.00), Math.toRadians(361.29))
                .splineTo(new Vector2d(2.97, -37.38), Math.toRadians(-2.86))
                .build();


        waitForStart();
        if (gamepad1.a) {
            drive.followTrajectorySequence(traj1);
        }
        else if (gamepad1.b) {
            stop();
        }
    }
}z