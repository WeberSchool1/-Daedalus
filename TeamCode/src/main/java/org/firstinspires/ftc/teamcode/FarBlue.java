package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Audience3ball", group="Autonomous")
public class FarBlue extends LinearOpMode {
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
    private Servo elevator, leftLed;
    private Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    private IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        elevator = hardwareMap.get(Servo.class, "turret");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftLed = hardwareMap.get(Servo.class, "LEDLeft");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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

        limelight.pipelineSwitch(4);
        waitForStart();
        limelight.start();

        // --- Limelight Turret auto Align ---
        LLResult ll;
        double tx;
        do {
            ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            tx = targetVisible ? ll.getTx() : 0.0;

            // Turret alignment
            if (targetVisible && Math.abs(tx) > deadbandDeg) {
                double turretPower = kP_TURRET * tx;
                turretPower = Math.max(-0.5, Math.min(turretPower, 0.5));
                turretSpin.setPower(turretPower);
            } else {
                turretSpin.setPower(0.0);
            }

            // LED indicator
            leftLed.setPosition(targetVisible ? 0.611 : 0);

            sleep(20); // small delay for CPU
        } while (Math.abs(tx) > deadbandDeg && opModeIsActive());

            // --- 1️⃣ Spin up shooter motor ---
            double targetPower = 0.65;                   // flywheel power level
            shooterMotor.setPower(targetPower);

            sleep(3000);

            // --- 3️⃣ Lift FIRST ball ---
            telemetry.addLine("Lifting first ball...");
            telemetry.update();
            elevator.setPosition(0);   // lift first ball up to shooter
            sleep(1000);
            elevator.setPosition(.6);

            sleep(1000);

            // --- 4️⃣ Reload sequence: use frontIntake to load next ball ---
            telemetry.addLine("Reloading second ball...");
            telemetry.update();
            frontIntake.setPower(1.0);
            sleep(1500);   // let front intake load onto elevator
            frontIntake.setPower(0.0);

            // --- 5️⃣ Wait for elevator to reset ---
            sleep(500);

            // --- 6️⃣ Lift SECOND ball ---
            telemetry.addLine("Feeding second ball...");
            telemetry.update();
            elevator.setPosition(0);   // lift second ball
            sleep(1000);
            elevator.setPosition(.6);

            // --- Feed Third Ball ---
            backIntake.setPower(1);
            sleep(1000);
            backIntake.setPower(0);
            // for rid of the sleep functoion her see what that chnages

            elevator.setPosition(0);
            sleep(1000);
            elevator.setPosition(.6);

            // --- 7️⃣ Optional: stop shooter and drive forward ---
            sleep(500);
            shooterMotor.setPower(0.0);


            // --- goes Forward to first row ---
            double drivePower = 0.25;
            frontLeft.setPower(drivePower);
            frontRight.setPower(drivePower);
            backLeft.setPower(drivePower);
            backRight.setPower(drivePower);
            sleep(2400);

            // -- turns left to face balls ---
            frontLeft.setPower(.4);
            frontRight.setPower(-.4);
            backLeft.setPower(.4);
            backRight.setPower(-.4);
            sleep(800);

            // goes forward to the balls and intakes --
            frontIntake.setPower(1);
            frontLeft.setPower(.4);
            frontRight.setPower(.4);
            backLeft.setPower(.4);
            backRight.setPower(.4);
            sleep(3500);

            // --- goes back wards Fast ---
            frontLeft.setPower(-.8);
            frontRight.setPower(-.8);
            backLeft.setPower(-.8);
            backRight.setPower(-.8);
            sleep(1000);

            // turns right to go to shooting zone --
            frontLeft.setPower(-.3);
            frontRight.setPower(.3);
            backLeft.setPower(-.3);
            backRight.setPower(.3);
            sleep(300);

            // --- goes back to shooting zone ---
            frontLeft.setPower(-.2);
            frontRight.setPower(-.2);
            backLeft.setPower(-.2);
            backRight.setPower(-.2);
            sleep(100);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            shooterMotor.setPower(.65);


            // --- 3️⃣ Feed FIRST ball ---


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



