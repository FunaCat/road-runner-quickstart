package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.TimeUnit;


public class AutonomousBlue2 extends LinearOpMode {
    public static class BLUEIDENTIFICATION implements VisionProcessor {
        Mat mixture_LEFT = new Mat();
        Mat mixture_MIDDLE = new Mat();
        Mat mixture_RIGHT = new Mat();
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

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    @Override
    public void runOpMode() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(
                WebcamName.class, "Webcam 1"), blueIdentificationProcess, tagProcessor);
        visionPortal.setProcessorEnabled(blueIdentificationProcess, false);

        waitForStart();

        tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);

        while (opModeIsActive()) {
            boolean aprilTagNotFound = false;
            double ANGLE = 30;
            SampleMecanumDrive drive;
            drive = new SampleMecanumDrive(hardwareMap);
            //spin to find april tags

            do {
                aprilTagNotFound = checkCoords(tagProcessor, aprilTagNotFound);
                drive.turn(Math.toRadians(ANGLE));
            } while (aprilTagNotFound);

            //depending on team head to these locations using RR
            Pose2d blue2 = new Pose2d(-36, 54, Math.toRadians(270));

            // TODO Fill out pose estimate with the correct position
            drive.setPoseEstimate();


            visionPortal.setProcessorEnabled(blueIdentificationProcess, true);
            propDetected();
            if (!propDetectedBoolean()) {
                propDetected();
            }
        }
    }

    public boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    public double calculatePositionX (double IDx, double Apriltagx) {
        return IDx - Apriltagx;
    }

    public double calculatePositionY(double IDy, double Apriltagy) {
        return IDy - Apriltagy;
    }

    public boolean checkCoords (AprilTagProcessor tagProcessor, boolean aprilTagNotFound) {
        setManualExposure(6, 250);
        if (tagProcessor.getDetections().size() > 0) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            switch (tag.metadata.id) {
                case 1: {
                    coordinateX = calculatePositionX(60, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(-42, tag.ftcPose.x) + Y_SHIFT);
                    heading = 90;
                    aprilTagNotFound = false;
                    break;
                }
                case 2: {
                    coordinateX = calculatePositionX(60, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(-36, tag.ftcPose.x) + Y_SHIFT);
                    heading = 90;
                    aprilTagNotFound = false;
                    break;
                }
                case 3: {
                    coordinateX = calculatePositionX(60, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(-30, tag.ftcPose.x) + Y_SHIFT);
                    heading = 90;
                    aprilTagNotFound = false;
                    break;
                }
                case 4: {
                    coordinateX = calculatePositionX(60, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(30, tag.ftcPose.x) + Y_SHIFT);
                    heading = 90;
                    aprilTagNotFound = false;
                    break;
                }
                case 5: {
                    coordinateX = calculatePositionX(60, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(36, tag.ftcPose.x) + Y_SHIFT);
                    heading = 90;
                    aprilTagNotFound = false;
                    break;
                }
                case 6: {
                    coordinateX = calculatePositionX(60, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(42, tag.ftcPose.x) + Y_SHIFT);
                    heading = 90;
                    aprilTagNotFound = false;
                    break;
                }
                case 7: {
                    coordinateX = calculatePositionX(-72, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(42, tag.ftcPose.x) + Y_SHIFT);
                    heading = 270;
                    aprilTagNotFound = false;
                    break;
                }
                case 10: {
                    coordinateX = calculatePositionX(-72, tag.ftcPose.y);
                    coordinateY = (calculatePositionY(-42, tag.ftcPose.x) + Y_SHIFT);
                    heading = 270;
                    aprilTagNotFound = false;
                    break;
                }
                default: {
                    aprilTagNotFound = true;
                    break;
                }
            }
        }
        setManualExposure(50, 100);
        return aprilTagNotFound;
    }

    public void propDetected() { //returns true if drops
        if (blueIdentificationProcess.propLeft) {
            //bearing 90
            //drop pixel
        } else if (blueIdentificationProcess.propMiddle) {
            // bearing = 0
            //drop pixel
        } else if (blueIdentificationProcess.propRight) {
            //bearing = 270
            //drop pixel
        }
    }

    public boolean propDetectedBoolean() { //returns true if drops
        if (blueIdentificationProcess.propLeft) {
            //bearing 90
            //drop pixel
            return true;
        } else if (blueIdentificationProcess.propMiddle) {
            // bearing = 0
            //drop pixel
            return true;
        } else if (blueIdentificationProcess.propRight) {
            //bearing = 270
            //drop pixel
            return true;
        }  else {
            return false;
        }
    }

    public void pickUp() {
        int runningautopixel;

        armBase = hardwareMap.get(DcMotor.class, "armBase");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armExtension = hardwareMap.get(DcMotor.class, "armExtension");
        claw = hardwareMap.get(Servo.class, "claw");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            runningautopixel = 1;
            armExtension.setTargetPosition(-14);
            claw.setPosition(0.78);
            while (runningautopixel == 1) {
                if (armExtension.getCurrentPosition() == -14) {
                    if (claw.getPosition() == 0.78) {
                        wrist.setPosition(0.9);
                        if (wrist.getPosition() == 0.9) {
                            armBase.setTargetPosition(0);
                            if (armBase.getCurrentPosition() == 0) {
                                claw.setPosition(0.9);
                                if (claw.getPosition() == 0.9) {
                                    runningautopixel = 0;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    public void dropOff(int target) { //1 = board 2 = spikes
        int xtarget;
        int runningautodropoff;
        // TODO: Enter the type for variable named target
        int ytarget;
        int targetbearing;
        // TODO: Enter the type for variable named robotbearing
        double robotbearing = //bearing RR;
        // TODO: Enter the type for variable named robotx
        double robotx = //xcoordinate RR;
        // TODO: Enter the type for variable named roboty
        double roboty = //y coordinate RR;

                armExtension = hardwareMap.get(DcMotor.class, "armExtension");
        armBase = hardwareMap.get(DcMotor.class, "armBase");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        // Put initialization blocks here.
        // set target as 1 for the board, and 2 for the spikes

        // only run this when the robot has the pixel (using autopickup
        runningautodropoff = 1;
        while (runningautodropoff == 1) {
            // update robotx, roboty, and robotbearing here
            if (target == 1) {
                xtarget = 0;
                ytarget = 0;
                targetbearing = 0;
                // move to the board (set x and y target to the board pos and move to it using roadrunner)
                if (robotx == xtarget && roboty == ytarget && robotbearing == targetbearing) {
                    armExtension.setTargetPosition(0);
                    armBase.setTargetPosition(0);
                    wrist.setPosition(0);
                    if (armBase.getCurrentPosition() == 0 && armExtension.getCurrentPosition() == 0 && wrist.getPosition() == 0) {
                        claw.setPosition(0);
                        if (claw.getPosition() == 0) {
                            runningautodropoff = 0;
                            break;
                        }
                    }
                }
            } else if (target == 2) {
                xtarget = 0;
                ytarget = 0;
                // move to the spike (set x and y target to the spike pos and move to it using roadrunner)
                if (robotx == xtarget && roboty == ytarget && robotbearing == targetbearing) {
                    armExtension.setTargetPosition(0);
                    armBase.setTargetPosition(0);
                    wrist.setPosition(0);
                    if (armBase.getCurrentPosition() == 0 && armExtension.getCurrentPosition() == 0 && wrist.getPosition() == 0) {
                        claw.setPosition(0);
                        if (claw.getPosition() == 0) {
                            runningautodropoff = 0;
                            break;
                        }
                    }
                }
            }
        }
    }
}


