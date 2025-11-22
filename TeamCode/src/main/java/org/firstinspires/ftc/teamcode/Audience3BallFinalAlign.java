package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Audience3Ball_FinalAlign", group="Autonomous")
public class Audience3BallFinalAlign extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, turretSpin, frontIntake, backIntake;
    private Servo elevator, leftLed;
    private Limelight3A limelight;

    private final double kP_TURRET = 0.02;
    private final double DEADZONE_TURRET = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware init ---
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        turretSpin   = hardwareMap.get(DcMotor.class, "turretOne");
        frontIntake  = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake   = hardwareMap.get(DcMotor.class, "backIntake");
        elevator     = hardwareMap.get(Servo.class, "turret");
        leftLed      = hardwareMap.get(Servo.class, "LEDLeft");
        limelight    = hardwareMap.get(Limelight3A.class, "limelight");

        // --- Motor directions ---
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight.pipelineSwitch(4);
        limelight.start();

        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();

        // --- 1️⃣ Spin up shooter ---
        shooterMotor.setPower(0.65);
        sleep(1500);

        // --- 2️⃣ Feed first ball ---
        elevator.setPosition(0);
        sleep(1000);
        elevator.setPosition(0.6);
        sleep(500);

        // --- 3️⃣ Load second ball with front intake ---
        frontIntake.setPower(1.0);
        sleep(1500);
        frontIntake.setPower(0);
        elevator.setPosition(0);
        sleep(500);
        elevator.setPosition(0.6);

        // --- 4️⃣ Load third ball with back intake ---
        backIntake.setPower(1.0);
        sleep(1000);
        backIntake.setPower(0);
        elevator.setPosition(0);
        sleep(500);
        elevator.setPosition(0.6);

        // --- 5️⃣ Perform movements (forward/back/turn as in original auto) ---
        // Example: drive forward
        frontLeft.setPower(0.25);
        frontRight.setPower(0.25);
        backLeft.setPower(0.25);
        backRight.setPower(0.25);
        sleep(2400);
        stopDriveMotors();

        // Add additional movements here (turns, strafes, etc.)
        // Example turn left:
        frontLeft.setPower(0.4);
        frontRight.setPower(-0.4);
        backLeft.setPower(0.4);
        backRight.setPower(-0.4);
        sleep(800);
        stopDriveMotors();

        // Example forward to balls and intake
        frontIntake.setPower(1.0);
        frontLeft.setPower(0.4);
        frontRight.setPower(0.4);
        backLeft.setPower(0.4);
        backRight.setPower(0.4);
        sleep(2500);
        frontIntake.setPower(0);
        stopDriveMotors();

        // --- 6️⃣ Final turret auto-align at end position ---
        LLResult ll;
        double tx;
        do {
            ll = limelight.getLatestResult();
            boolean targetVisible = (ll != null && ll.isValid());
            tx = targetVisible ? ll.getTx() : 0.0;

            double turretPower = 0;
            if (targetVisible && Math.abs(tx) > DEADZONE_TURRET) {
                turretPower = clamp(kP_TURRET * tx, -0.5, 0.5);
            }
            turretSpin.setPower(turretPower);

            leftLed.setPosition(targetVisible ? 0.611 : 0);

            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("TX", tx);
            telemetry.update();
            sleep(20);
        } while (Math.abs(tx) > DEADZONE_TURRET && opModeIsActive());

        // --- Stop all motors ---
        shooterMotor.setPower(0);
        turretSpin.setPower(0);
        frontIntake.setPower(0);
        backIntake.setPower(0);
        stopDriveMotors();

        telemetry.addLine("Auto complete! Turret aligned.");
        telemetry.update();
    }

    private void stopDriveMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
