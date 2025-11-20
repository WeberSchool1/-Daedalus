package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Audience3ball_Pinpoint", group="Autonomous")
public class FarBlue_Pinpoint extends LinearOpMode {

    private final double kP_TURRET = 0.02;
    private final double deadbandDeg = 0.5;

    private final double X_POD_OFFSET = 0.585; // inches
    private final double Y_POD_OFFSET = 0.875; // inches

    private final double MAX_DRIVE_POWER = 0.8;
    private final double MAX_TURN_POWER = 0.3;

    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, turretSpin, backIntake, frontIntake;
    private Servo elevator, leftLed;
    private Limelight3A limelight;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware init ---
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        elevator = hardwareMap.get(Servo.class, "turret");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        leftLed = hardwareMap.get(Servo.class, "LEDLeft");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Pinpoint configuration ---
        pinpoint.setOffsets(.585, .875, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        limelight.pipelineSwitch(4);

        telemetry.addLine("Ready to run");
        telemetry.update();
        waitForStart();
        limelight.start();

        // --- 1️⃣ Auto-align turret at the start ---
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

            sleep(20);
        } while (Math.abs(tx) > deadbandDeg && opModeIsActive());

        // --- 2️⃣ Spin up shooter ---
        shooterMotor.setPower(0.65);
        sleep(3000);

        // --- 3️⃣ Feed balls (same as your current auto) ---
        // First ball
        elevator.setPosition(0);
        sleep(1000);
        elevator.setPosition(0.6);
        sleep(1000);

        // Second ball
        frontIntake.setPower(1.0);
        sleep(1500);
        frontIntake.setPower(0.0);

        elevator.setPosition(0);
        sleep(500);
        elevator.setPosition(0.6);

        // Third ball
        backIntake.setPower(1.0);
        sleep(1000);
        backIntake.setPower(0.0);

        elevator.setPosition(0);
        sleep(1000);
        elevator.setPosition(0.6);

        shooterMotor.setPower(0.0);

        // --- 4️⃣ Drive sequence using Pinpoint ---
        // Example placeholders — replace targetX/targetY with real field coordinates later
        moveToPosition(30, 0, MAX_DRIVE_POWER); // move forward to first row
        turnToHeading(90, MAX_TURN_POWER); // turn left
        moveToPosition(30, 20, MAX_DRIVE_POWER); // drive forward while intaking
        frontIntake.setPower(1.0);
        sleep(3500);
        frontIntake.setPower(0.0);

        moveToPosition(10, 20, MAX_DRIVE_POWER); // go backward fast
        turnToHeading(0, MAX_TURN_POWER); // turn right
        moveToPosition(0, 0, MAX_DRIVE_POWER); // back to shooting zone

        stopAllMotors();

        telemetry.addLine("All done!");
        telemetry.update();
    }

    // --- Helper functions ---
    private void moveToPosition(double targetX, double targetY, double maxPower) {
        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();
            double dx = targetX - pos.getX(DistanceUnit.INCH);
            double dy = targetY - pos.getY(DistanceUnit.INCH);

            // Simple proportional drive (can be improved with PID)
            double powerX = clamp(dx * 0.05, -maxPower, maxPower);
            double powerY = clamp(dy * 0.05, -maxPower, maxPower);

            // For mecanum drive: forward/back = Y, strafe = X
            frontLeft.setPower(powerY + powerX);
            backLeft.setPower(powerY - powerX);
            frontRight.setPower(powerY - powerX);
            backRight.setPower(powerY + powerX);

            // Stop if close enough (±0.5 inches)
            if (Math.abs(dx) < 0.5 && Math.abs(dy) < 0.5) break;

            sleep(20);
        }

        stopAllMotors();
    }

    private void turnToHeading(double targetHeading, double maxTurnPower) {
        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();
            double error = targetHeading - pos.getHeading(AngleUnit.DEGREES);

            // Normalize error to -180..180
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double power = clamp(error * 0.01, -maxTurnPower, maxTurnPower);

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            if (Math.abs(error) < 2.0) break; // within 2 degrees
            sleep(20);
        }
        stopAllMotors();
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontIntake.setPower(0);
        backIntake.setPower(0);
        shooterMotor.setPower(0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
