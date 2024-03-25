package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@Autonomous
public class RRTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d initPosition = new Pose2d(-48, 0, Math.toRadians(0));
        drive.trajectoryBuilder(initPosition);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(initPosition)
                .splineTo(new Vector2d(-19.19, -0.00, Math.toRadians(361.29))
                .splineTo(new Vector2d(2.97, -37.38, Math.toRadians(-2.86))
                .build());


        waitForStart();
        if (gamepad1.a) {
            drive.followTrajectorySequence(traj1);
        }
        else if (gamepad1.b) {
            stop();
        }
    }
}