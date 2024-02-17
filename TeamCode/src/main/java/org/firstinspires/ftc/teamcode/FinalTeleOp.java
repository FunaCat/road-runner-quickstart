package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "FinalTeleOp")
public class FinalTeleOp extends LinearOpMode {

    private Servo drone;
    private IMU imu;

    // mecanum wheels
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;

    // arm and claw
    private DcMotor armBase;
    private DcMotor armExtension;
    private Servo claw;
    private Servo wrist;

    // suspension
    private DcMotor winch;
    private Servo winchLock;

    /**
     * Describe this function...
     */
    private void drone() {
        if (gamepad2.b) {
            drone.setPosition(0.35);
        } else {
            drone.setPosition(0.1);
        }
    }

    /**
     * Describe this function...
     */
    private void mecanumSetup() {
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    private void armSetup() {
        armBase.setTargetPosition(armBase.getCurrentPosition());
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setPower(1);

        armExtension.setTargetPosition(armExtension.getCurrentPosition());
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setPower(1);
    }

    private void suspensionSetup() {
        winchLock.setPosition(1);

        winch.setTargetPosition(winch.getCurrentPosition());
        winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setPower(1);
    }

    /**
     * Describe this function...
     */
    private void droneSetup() {
        drone.setPosition(0.1);
    }

    private void clawSetup() {
        claw.setPosition(0);

        wrist.setPosition(0);
    }

    private void hubSetup() {
        drone = hardwareMap.get(Servo.class, "drone");
        imu = hardwareMap.get(IMU.class, "imu");

        // mecanum wheels
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        // arm and claw
        armBase = hardwareMap.get(DcMotor.class, "armBase");
        armExtension = hardwareMap.get(DcMotor.class, "armExtension");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        //Suspension
        winch = hardwareMap.get(DcMotor.class, "winch");
        winchLock = hardwareMap.get(Servo.class, "winchLock");
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        hubSetup();
        mecanumSetup();
        armSetup();
        droneSetup();
        suspensionSetup();
        clawSetup();

        waitForStart();

        if (!isStopRequested() && opModeIsActive()) {
            while (opModeIsActive()) {
                // MAIN Loop
                manualMove();
                armBase();
                armExtend();
                drone();
                claw();
                winch();
                telemetry.addData("wrist", wrist.getPosition());
                telemetry.addData("claw", claw.getPosition());
                telemetry.addData("winchLock", winchLock.getPosition());
                telemetry.addData("armExtend", armExtension.getCurrentPosition());
                telemetry.addData("drone", drone.getPosition());
                telemetry.update();
            }
        }
    }

    private void claw() {
        //finger add control
        if (gamepad1.right_stick_y < 0) {
            claw.setPosition(claw.getPosition() + .01);
        } else if (gamepad1.right_stick_y > 0) {
            claw.setPosition(claw.getPosition() - .01);
        }
        //wrist
        if (gamepad1.right_bumper) {
            wrist.setPosition(wrist.getPosition() + .01);
        } else if (Math.abs(gamepad1.right_trigger) > 0) {
            wrist.setPosition(wrist.getPosition() - .01);
        }
    }


    private void winch(){
        //winch
        if (gamepad1.a) {
            winch.setTargetPosition(40 + winch.getCurrentPosition());
        } else if (gamepad1.y) {
            winch.setTargetPosition(-40 + winch.getCurrentPosition());
        }
        //winchLock
        if (gamepad1.b) {
            winchLock.setPosition(0.5);
        } else {
            winchLock.setPosition(1);
        }
    }

    private void armBase() {
        if (gamepad1.left_bumper) {
            armBase.setTargetPosition(40 + armBase.getCurrentPosition());
        } else if (Math.abs(gamepad1.left_trigger) > 0) {
            armBase.setTargetPosition(-40 + armBase.getCurrentPosition());
        }
        //sorry again
        if (gamepad1.dpad_up) {
            winch.setTargetPosition(40 + winch.getCurrentPosition());
        } else if (gamepad1.dpad_down) {
            winch.setTargetPosition(-40 + winch.getCurrentPosition());
        } else {
            winch.setTargetPosition(winch.getCurrentPosition());
        }
    }

    /**
     * Describe this function...
     */
    private void armExtend() {
        int extensionIntensity = 30;
        if (gamepad1.left_stick_y < 0) {
            armExtension.setTargetPosition(-extensionIntensity + armExtension.getCurrentPosition());
        } else if (gamepad1.left_stick_y > 0) {
            armExtension.setTargetPosition(extensionIntensity + armExtension.getCurrentPosition());
        }
    }

    /**
     * Describe this function...
     */
    private void manualMove() {
        float y;
        float x;
        float t;
        double botHeading;
        double rotX;
        double rotY;
        double denominator;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        y = gamepad2.left_stick_y;
        x = -gamepad2.left_stick_x;
        t = -gamepad2.right_stick_x;
        if (gamepad2.start) {
            imu.resetYaw();
        }
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        rotX = x * Math.cos(-botHeading / 180 * Math.PI) - y * Math.sin(-botHeading / 180 * Math.PI);
        rotY = x * Math.sin(-botHeading / 180 * Math.PI) + y * Math.cos(-botHeading / 180 * Math.PI);
        rotX = rotX * 1.1;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(rotY) + Math.abs(rotX) + Math.abs(t), 1));
        frontLeftPower = (rotY + rotX + t) / denominator;
        backLeftPower = (rotY - (rotX - t)) / denominator;
        frontRightPower = (rotY - (rotX + t)) / denominator;
        backRightPower = (rotY + (rotX - t)) / denominator;
        frontLeft.setPower(0.75 * frontLeftPower);
        backLeft.setPower(0.75 * backLeftPower);
        frontRight.setPower(0.75 * frontRightPower);
        backRight.setPower(0.75 * backRightPower);
    }
}

