package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.clamp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="AutoTwolaunch", group="Autonomous")
public class Autotwolaucnh extends LinearOpMode {
    private final double kP_TURRET = 0.02;
    private final double ALIGN_TOLERANCE = 1.0; // degrees
    private final double CAMERA_HEIGHT = 12.54; // inches
    private final double TARGET_HEIGHT = 36.0;  // top of AprilTag
    private final double CAMERA_ANGLE = 70.0;   // degrees up
    private final double MIN_DISTANCE = 20.0;
    private final double MAX_DISTANCE = 100.0;
    private final double MIN_POWER = 0.45;
    private final double MAX_POWER = 0.75;
    private final long SHOOTER_SPINUP_MS = 800;
    private final long FEED_DURATION_MS = 700;

    private final double deadbandDeg = 0.5;


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, turretSpin, backIntake, frontIntake;
    private Servo turretHood;
    private Limelight3A limelight;
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // --- setup ---
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        turretHood.setPosition(.5);

        LLResult ll = limelight.getLatestResult();
        boolean targetVisible = (ll != null && ll.isValid());
        double tx = targetVisible ? ll.getTx() : 0.0;
        double ty = targetVisible ? ll.getTy() : 0.0;

        // --- 1️⃣ Spin up shooter motor ---
        if (targetVisible) {
            if (Math.abs(tx) > deadbandDeg) {
                double turretPower = kP_TURRET * tx;
                turretPower = clamp(turretPower, -0.5, 0.5);
                turretSpin.setPower(turretPower);
            } else {
                turretSpin.setPower(0.0);
            }
        } else {
            turretSpin.setPower(0.0);
        }


        // --- Shooter auto-speed ---
        double targetShooterPower = 0.0; // initialize target power

        if (gamepad1.y) {
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                    Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
            distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
            targetShooterPower = MIN_POWER + (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (MAX_POWER - MIN_POWER);
            shooterMotor.setPower(targetShooterPower);
        } else {
            shooterMotor.setPower(0.0);
            targetShooterPower = 0.0;
        }
    }
}


