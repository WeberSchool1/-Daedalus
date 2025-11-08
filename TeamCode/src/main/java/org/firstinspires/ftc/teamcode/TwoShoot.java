package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.clamp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


@Autonomous(name="Twoshoot", group="Autonomous")
public class TwoShoot extends LinearOpMode {
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
    private Limelight3A limelight;
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake"); // elevator
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake"); // ground intake
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

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        // --- 1️⃣ Spin up shooter motor ---
        double targetPower = 0.8;                   // flywheel power level
        shooterMotor.setPower(targetPower);
        double targetVelocity = 6000 * targetPower; // estimate in ticks/sec
        double tolerance = 0.05;                    // 5% tolerance

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

        // --- 3️⃣ Feed FIRST ball ---
        telemetry.addLine("Feeding first ball...");
        telemetry.update();
        backIntake.setPower(1.0);   // lift first ball up to shooter
        sleep(1200);
        backIntake.setPower(0.0);

        // --- 4️⃣ Reload sequence: use frontIntake to load next ball ---
        telemetry.addLine("Reloading second ball...");
        telemetry.update();
        frontIntake.setPower(1.0);  // pick up from ground
        sleep(1200);                // let front intake load onto elevator
        frontIntake.setPower(0.0);

        // --- 5️⃣ Wait for elevator to reset ---
        sleep(1500);

        // --- 6️⃣ Feed SECOND ball ---
        telemetry.addLine("Feeding second ball...");
        telemetry.update();
        backIntake.setPower(1.0);   // lift second ball
        sleep(1200);
        backIntake.setPower(0.0);

        // --- 7️⃣ Optional: stop shooter and drive forward ---
        sleep(500);
        shooterMotor.setPower(0.0);

        double drivePower = 0.3;
        frontLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backLeft.setPower(drivePower);
        backRight.setPower(drivePower);
        sleep(3000);

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


