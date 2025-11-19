package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.clamp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Audience3ball", group="Autonomous")
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
    private Servo elevator;
    private Limelight3A limelight;
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        elevator = hardwareMap.get(Servo.class, "turret");
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
        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        turretSpin.setPower(0);

        LLResult ll = limelight.getLatestResult();
        boolean targetVisible = (ll != null && ll.isValid());
        double tx = targetVisible ? ll.getTx() : 0.0;
        double ty = targetVisible ? ll.getTy() : 0.0;

        // --- 1️⃣ Spin up shooter motor ---
        double targetPower = 0.65;                   // flywheel power level
        shooterMotor.setPower(targetPower);

        sleep(3000);

        // --- 3️⃣ Feed FIRST ball ---
        telemetry.addLine("Feeding first ball...");
        telemetry.update();
        elevator.setPosition(0);   // lift first ball up to shooter
        sleep(1000);
        elevator.setPosition(.6);

        sleep(2000);

        // --- 4️⃣ Reload sequence: use frontIntake to load next ball ---
        telemetry.addLine("Reloading second ball...");
        telemetry.update();
        frontIntake.setPower(1.0);  // pick up from ground
        sleep(1500);                // let front intake load onto elevator
        frontIntake.setPower(0.0);

        // --- 5️⃣ Wait for elevator to reset ---
        sleep(1000);

        // --- 6️⃣ Feed SECOND ball ---
        telemetry.addLine("Feeding second ball...");
        telemetry.update();
       elevator.setPosition(0);   // lift second ball
        sleep(1000);
        elevator.setPosition(.6);

        backIntake.setPower(.9);
        sleep(1500);
        backIntake.setPower(0);

        sleep(1000);

        elevator.setPosition(0);
        sleep(1200);
        elevator.setPosition(.6);

        // --- 7️⃣ Optional: stop shooter and drive forward ---
        sleep(500);
        shooterMotor.setPower(0.0);

        double drivePower = 0.27;
        frontLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backLeft.setPower(drivePower);
        backRight.setPower(drivePower);
        sleep(2400);

        frontLeft.setPower(.4);
        frontRight.setPower(-.4);
        backLeft.setPower(.4);
        backRight.setPower(-.4);
        sleep(900);

        frontLeft.setPower(.4);
        frontRight.setPower(.4);
        backLeft.setPower(.4);
        backRight.setPower(.4);
        sleep(1500);


        // --- 8️⃣ Stop all ---
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooterMotor.setPower(0);
        frontIntake.setPower(0);
        backIntake.setPower(0);

        telemetry.addLine("All done!");
        telemetry.update();
    }
}


