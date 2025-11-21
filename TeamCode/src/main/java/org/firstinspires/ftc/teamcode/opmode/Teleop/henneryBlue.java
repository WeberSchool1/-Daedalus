package org.firstinspires.ftc.teamcode.opmode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Configurable
@TeleOp(name = "henneryBlue", group = "TeleOp")
public class henneryBlue extends LinearOpMode {
    private static double DRIVE_SCALE = 0.85;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake, backIntake, turretSpin;
    private Servo turretHood, rightLed, leftLed, elevator,rightPTO,leftPTO;
    private Limelight3A limelight;

    // Turret auto-align constants
    private static double kP_TURRET = 0.02;
    private static double deadbandDeg = 0.5;

    // Shooter auto-speed constants
    private final double CAMERA_HEIGHT = 12.54; // inches
    private final double TARGET_HEIGHT = 29.5;  // example: hub height
    private final double CAMERA_ANGLE = 19.0;   // degrees
    private static double MIN_DISTANCE = 20.0;   // inches
    private static double MAX_DISTANCE = 140.0;  // inches
    private static double MIN_POWER = 0.45;
    private static double MAX_POWER = 0.78;
    private static int slowdown = 1;

    @Override
    public void runOpMode() throws InterruptedException {



        // --- Hardware mapping ---
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");


        rightPTO = hardwareMap.get(Servo.class,"rightPTO");
        leftPTO = hardwareMap.get(Servo.class,"rightPTO");
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        elevator = hardwareMap.get(Servo.class, "turret");
        leftLed = hardwareMap.get(Servo.class, "LEDLeft");
        rightLed = hardwareMap.get(Servo.class, "LEDRight");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);

        telemetry.addLine("henneryFinal initialized. Press start.");
        telemetry.update();

        waitForStart();
        limelight.start();

        rightPTO.setPosition(.075);
        leftPTO.setPosition(.045);

        while (opModeIsActive()) {
            // --- Drive controls ---
            double driveY = -gamepad1.left_stick_y * DRIVE_SCALE;
            double driveX = gamepad1.left_stick_x * DRIVE_SCALE;
            double turn = gamepad1.right_stick_x * DRIVE_SCALE;

            double fl = (driveY - driveX + turn)/slowdown;
            double fr = (driveY + driveX - turn)/slowdown;
            double bl = (driveY + driveX + turn)/slowdown;
            double br = (driveY - driveX - turn)/slowdown;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // --- Limelight read ---
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0.0;
            double ty = targetVisible ? ll.getTy() : 0.0;

            // --- Turret auto-align ---
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

            if (gamepad1.b) {
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
                distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
                targetShooterPower = MIN_POWER + (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (MAX_POWER - MIN_POWER);
                shooterMotor.setPower(targetShooterPower);
            } else if (gamepad1.x) {
                shooterMotor.setPower(0.0);
                targetShooterPower = 0.0;
            }

// --- Intake controls ---

            if (gamepad1.right_bumper) {
                frontIntake.setPower(-.3);
            }
            else {
                frontIntake.setPower(gamepad1.right_trigger);}

            if (gamepad1.left_bumper) {
                backIntake.setPower(-.3);
            }
            else {
                backIntake.setPower(gamepad1.left_trigger);}


// --- Turret hood ---
            if (gamepad1.dpad_up) {
                turretHood.setPosition(0.8);
            }
            if (gamepad1.dpad_down) {
                turretHood.setPosition(0.45);}

            if (gamepad1.dpad_left){
                turretSpin.setPower(-.3);
            }
            if (gamepad1.dpad_right){
                turretSpin.setPower(.3);
            }
//--- drive slower ---
            if (gamepad1.y) {
                slowdown = 3;
            }
            else {
                slowdown = 1;}

// --- LED feedback ---
            if (targetVisible){
                rightLed.setPosition(.611);
                leftLed.setPosition(.611);}
            else {
                leftLed.setPosition(0);
            rightLed.setPosition(0);}
            
           

            // ----Elevator up-----
            if(gamepad1.a){
                elevator.setPosition(.0);
            }else {
                elevator.setPosition(.5);
            }

            if(gamepad1.dpad_up){
                rightPTO.setPosition(.075);
                leftPTO.setPosition(.045);
            } if (gamepad1.dpad_down){
                rightPTO.setPosition(.55);
                leftPTO.setPosition(.49);
            }
// --- Telemetry ---

            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.addData("Target Shooter Power", targetShooterPower);

// Shooter up-to-speed check (tolerance 0.02)
            boolean shooterAtSpeed = Math.abs(shooterMotor.getPower() - targetShooterPower) < 0.02;
            telemetry.addData("Shooter At Speed", shooterAtSpeed);

            
            telemetry.update();

        }
    }

    private double clamp(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}
