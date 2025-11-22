package org.firstinspires.ftc.teamcode.opmode.Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "RedFieldCentric", group = "TeleOp")
public class RedFieldCentric extends LinearOpMode {

    private static final double DRIVE_SCALE = 0.85;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake, backIntake, turretSpin;
    private Servo turretHood, rightLed, leftLed, elevator, rightPTO, leftPTO;
    private Limelight3A limelight;

    private IMU imu;

    private final double kP_TURRET = 0.02;
    private final double deadbandDeg = 0.5;

    private final double CAMERA_HEIGHT = 12.54;
    private final double TARGET_HEIGHT = 29.5;
    private final double CAMERA_ANGLE = 19.0;
    private final double MIN_DISTANCE = 20.0;
    private final double MAX_DISTANCE = 140.0;
    private final double MIN_POWER = 0.43;
    private final double MAX_POWER = 0.72;

    private int slowdown = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------ Hardware Map ------------
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
        leftPTO  = hardwareMap.get(Servo.class,"rightPTO"); // ⚠ check mapping
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        elevator = hardwareMap.get(Servo.class, "turret");
        leftLed = hardwareMap.get(Servo.class, "LEDLeft");
        rightLed = hardwareMap.get(Servo.class, "LEDRight");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);

        // ------------ IMU Setup (FIELD CENTRIC) ------------
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("RedFieldCentric Ready");
        telemetry.update();

        waitForStart();
        limelight.start();

        rightPTO.setPosition(.075);
        leftPTO.setPosition(.045);

        // ===================== TELEOP LOOP =====================
        while (opModeIsActive()) {

            // ---------------- FIELD-CENTRIC DRIVE (RED MIRROR) ----------------
            if (gamepad1.dpad_down) imu.resetYaw();   // reset heading

            double forward = -gamepad1.left_stick_y * DRIVE_SCALE;
            double strafe  =  gamepad1.left_stick_x * DRIVE_SCALE;
            double turn    =  gamepad1.right_stick_x * DRIVE_SCALE;

            // Convert to polar
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(strafe, forward);

            // Mirror for Red alliance: rotate by (yaw + 180°)
            theta += Math.toRadians(180) - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Back to cartesian
            double fwd = r * Math.cos(theta);
            double str = r * Math.sin(theta);

            driveRobotCentric(fwd / slowdown, str / slowdown, turn / slowdown);

            // ---------------- LIMELIGHT ----------------
            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            double tx = targetVisible ? ll.getTx() : 0;

            if (targetVisible) {
                double turretPower = (Math.abs(tx) > deadbandDeg) ? kP_TURRET * tx : 0;
                turretSpin.setPower(clamp(turretPower, -0.5, 0.5));
            } else {
                turretSpin.setPower(0);
            }

            if (targetVisible) {
                rightLed.setPosition(.611);
                leftLed.setPosition(.611);
            } else {
                leftLed.setPosition(0);
                rightLed.setPosition(0);
            }

            // Shooter auto-speed
            double shooterPowerTarget = 0;
            if (gamepad1.b) {
                double ty = ll != null ? ll.getTy() : 0;
                double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
                distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
                shooterPowerTarget = MIN_POWER +
                        (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE)
                                * (MAX_POWER - MIN_POWER);
                shooterMotor.setPower(shooterPowerTarget);
            } else if (gamepad1.x) {
                shooterMotor.setPower(0);
            }

            // ---------------- Intake & Turret ----------------
            frontIntake.setPower(gamepad1.right_bumper ? -0.3 : gamepad1.right_trigger);
            backIntake.setPower(gamepad1.left_bumper ? -0.3 : gamepad1.left_trigger);

            if (gamepad1.dpad_up) turretHood.setPosition(0.8);
            if (gamepad1.dpad_down) turretHood.setPosition(0.45);
            if (gamepad1.dpad_left) turretSpin.setPower(-0.3);
            if (gamepad1.dpad_right) turretSpin.setPower(0.3);

            slowdown = gamepad1.y ? 3 : 1;
            elevator.setPosition(gamepad1.a ? 0.0 : 0.5);

            telemetry.update();
        }
    }

    private void driveRobotCentric(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
