package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "shootinZoneTest", group = "TeleOp")
public class ShootInZoneTest extends LinearOpMode {

    private DcMotor turretSpin;
    private Limelight3A limelight;
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;

    private final double kP_TURRET = 0.02;
    private final double deadbandDeg = 0.5;

    // Shooting zone coordinates (adjust to your field)
    private final double shootZoneXMin = 100;
    private final double shootZoneXMax = 150;
    private final double shootZoneYMin = 50;
    private final double shootZoneYMax = 120;

    @Override
    public void runOpMode() throws InterruptedException {

        turretSpin = hardwareMap.get(DcMotor.class, "turretOne");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);
        limelight.start();

        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "pinpoint");
        // Set your odometry offsets
        pinpoint.setOffsets(-0.585, -0.875, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD,
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 1.25, 1, AngleUnit.DEGREES, 0));

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
            boolean inShootingZone = robotX >= shootZoneXMin && robotX <= shootZoneXMax &&
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

            telemetry.addData("In Shooting Zone", inShootingZone);
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("TX", tx);
            telemetry.update();
        }
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
