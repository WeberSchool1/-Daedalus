package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="closeblue", group="Autonomous")
public class closeblue extends LinearOpMode {
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
    private Servo turretHood, elevator;
    private Limelight3A limelight;
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        elevator = hardwareMap.get(Servo.class, "turret");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");



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

        turretHood.setPosition(.45);

        // --- Limelight read ---
        LLResult ll = limelight.getLatestResult();
        boolean targetVisible = (ll != null && ll.isValid());
        double tx = targetVisible ? ll.getTx() : 0.0;
        double ty = targetVisible ? ll.getTy() : 0.0;

        double drivePower = 0.3;
        frontLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backLeft.setPower(drivePower);
        backRight.setPower(drivePower);
        sleep(2005);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // --- Turret auto-align ---
        // --- 1️⃣ Spin up shooter motor ---
        double targetPower = 0.449;                   // flywheel power level
        shooterMotor.setPower(targetPower);
        double targetVelocity = 6000 * targetPower; // estimate in ticks/sec
        double tolerance = 0.11;                    // 5% tolerance

        int lastTicks = shooterMotor.getCurrentPosition();
        long lastTime = System.currentTimeMillis();
        double velocity = 0;

        // --- 2️⃣ Wait for shooter to reach target velocity ---
        while (opModeIsActive()) {
            int currentTicks = shooterMotor.getCurrentPosition();
            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt > 0) velocity = (currentTicks - lastTicks) / dt;

            lastTicks = currentTicks;
            lastTime = now;

            boolean upToSpeed = Math.abs(velocity - targetVelocity)
                    <= (targetVelocity * tolerance);

            telemetry.addData("Velocity (ticks/sec)", velocity);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Up to speed?", upToSpeed);
            telemetry.update();

            if (upToSpeed) break; // ready to shoot
        }

        sleep(600);

        // --- 3️⃣ Feed FIRST ball ---
        telemetry.addLine("Feeding first ball...");
        telemetry.update();
        elevator.setPosition(0);   // lift first ball up to shooter
        sleep(1000);
        elevator.setPosition(.6);

        // --- 4️⃣ Reload sequence: use frontIntake to load next ball ---
        telemetry.addLine("Reloading second ball...");
        telemetry.update();
        frontIntake.setPower(1.0);  // pick up from ground
        sleep(1000);                // let front intake load onto elevator
        frontIntake.setPower(0.0);

        // --- 6️⃣ Feed SECOND ball ---
        telemetry.addLine("Feeding second ball...");
        telemetry.update();
        elevator.setPosition(0);   // lift second ball
        sleep(1000);
        elevator.setPosition(.6);

        backIntake.setPower(1.0);
        sleep(1000);
        backIntake.setPower(0);

        sleep(500);

        elevator.setPosition(0);
        sleep(1500);
        elevator.setPosition(.6);

        // --- Strafe right for 1.5 seconds ---
        double strafePower = 0.5;

// Mecanum strafing configuration
        frontLeft.setPower(-strafePower);
        backLeft.setPower(strafePower);
        frontRight.setPower(strafePower);
        backRight.setPower(-strafePower);

        sleep(1500);

// Stop all drive motors
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);






        // --- 8️⃣ Stop all ---
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooterMotor.setPower(0);
        frontIntake.setPower(0);
        backIntake.setPower(0);

    }
}


