package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.opmode.Teleop.BlueFieldCentric.DRIVE_SCALE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "shootinZoneTest", group = "TeleOp")
public class shootInZoneTest extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor shooterMotor, frontIntake, backIntake, turretSpin;
    private Servo turretHood, rightLed, leftLed, elevator, rightPTO, leftPTO;
    private IMU imu;
    private Limelight3A limelight;
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;

    private final double kP_TURRET = 0.02;
    private final double deadbandDeg = 0.5;

    // Shooting zone coordinates (adjust to your field)
    private final double shootZoneXMin = 10;
    private final double shootZoneXMax = 75;
    private final double shootZoneYMin = 10;
    private final double shootZoneYMax = 75;
    private final double CAMERA_HEIGHT = 12.54;
    private final double TARGET_HEIGHT = 29.5;
    private final double CAMERA_ANGLE = 19.0;
    private final double MIN_DISTANCE = 20.0;
    private final double MAX_DISTANCE = 140.0;
    private final double MIN_POWER = 0.45;
    private final double MAX_POWER = 0.78;

    private int slowdown = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);
        limelight.start();

        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");
        // Set your odometry offsets
        pinpoint.setOffsets(0.585, 0.875, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD,
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

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
        limelight.pipelineSwitch(4);

        // ------------ IMU Setup (FIELD CENTRIC) ------------
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("BlueFieldCentric Ready");
        telemetry.update();

        telemetry.addLine("Ready to auto-align turret");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Update robot position
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);

            // Check if in shooting zone
            boolean inShootingZone = robotX >= shootZoneXMin && robotX <=shootZoneXMax&&
                                     robotY >= shootZoneYMin && robotY <= shootZoneYMax;

            LLResult ll = limelight.getLatestResult();
            boolean targetVisible = ll != null && ll.isValid();
            double tx = targetVisible ? ll.getTx() : 0.0;

            if (inShootingZone) {
                if (targetVisible) {
                    // Auto-align turret
                    double turretPower = Math.abs(tx) > deadbandDeg ? kP_TURRET * tx : 0;
                    turretSpin.setPower(clamp(turretPower, -0.5, 0.5));
                } else {
                    // Slowly rotate turret to find target
                    turretSpin.setPower(0.2);
                }
            } else {
                turretSpin.setPower(0);
            }
            // ------------ IMU Setup (FIELD CENTRIC) ------------
            imu = hardwareMap.get(IMU.class, "imu");

            orientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            );
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            telemetry.addLine("BlueFieldCentric Ready");
            telemetry.update();

            waitForStart();
            limelight.start();

            rightPTO.setPosition(.075);
            leftPTO.setPosition(.045);

                // ---------------- FIELD-CENTRIC DRIVE ----------------
                if (gamepad1.dpad_down) imu.resetYaw();   // reset heading

                double forward = -gamepad1.left_stick_y * DRIVE_SCALE;
                double strafe  =  gamepad1.left_stick_x * DRIVE_SCALE;
                double turn    =  gamepad1.right_stick_x * DRIVE_SCALE;

                // Convert to polar
                double theta = Math.atan2(forward, strafe);
                double r = Math.hypot(strafe, forward);

                // Rotate by robot heading
                theta -= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Back to cartesian
                double fwd = r * Math.cos(theta);
                double str = r * Math.sin(theta);

                driveRobotCentric(fwd / slowdown, str / slowdown, turn / slowdown);


                // Turret auto-align
                if (targetVisible) {
                    double turretPower = (Math.abs(tx) > deadbandDeg) ? kP_TURRET * tx : 0;
                    turretSpin.setPower(clamp(turretPower, -0.5, 0.5));
                } else {
                    turretSpin.setPower(0);
                }

                // LEDs
                if (targetVisible) {
                    rightLed.setPosition(.611);
                    leftLed.setPosition(.611);
                } else {
                    leftLed.setPosition(0);
                    rightLed.setPosition(0);
                }

                // ---------------- Shooter Auto Speed ----------------
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

                // ---------------- Intake Controls ----------------
                frontIntake.setPower(
                        gamepad1.right_bumper ? -0.3 : gamepad1.right_trigger
                );
                backIntake.setPower(
                        gamepad1.left_bumper ? -0.3 : gamepad1.left_trigger
                );

                // ---------------- Turret Hood ----------------

                // Manual turret override
                if (gamepad1.dpad_left)  turretSpin.setPower(-0.3);
                if (gamepad1.dpad_right) turretSpin.setPower(0.3);

                slowdown = gamepad1.y ? 3 : 1;

                // Elevator
                elevator.setPosition(gamepad1.a ? 0.0 : 0.5);

                telemetry.update();
            }
        }

        // ---------------- Helper: robot-centric drive ----------------a C
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
