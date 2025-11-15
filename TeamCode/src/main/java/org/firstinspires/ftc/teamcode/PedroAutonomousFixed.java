package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous - Fixed", group = "Autonomous")
@Configurable
public class PedroAutonomousFixed extends LinearOpMode {

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  private Follower follower; // Pedro Pathing follower instance
  private Paths paths; // Paths defined below

  @Override
  public void runOpMode() throws InterruptedException {
    // Panels telemetry (optional)
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // create follower (Constants.createFollower should return a configured Follower)
    follower = Constants.createFollower(hardwareMap);

    // Set starting pose exactly as you requested:
    // X = 25.540, Y = 129.718, heading = -43 degrees (convert to radians)
    Pose startPose = new Pose(25.540, 129.718, Math.toRadians(-43));
    follower.setStartingPose(startPose);

    // Build paths
    paths = new Paths(follower);

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);

    // Wait for start
    waitForStart();

    // Start following Path1
    // Start following Path1
    follower.followPath(paths.Path1);

// Follow until done
    while (opModeIsActive() && follower.isBusy()) {
      follower.update();

      panelsTelemetry.debug("X", follower.getPose().getX());
      panelsTelemetry.debug("Y", follower.getPose().getY());
      panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
      panelsTelemetry.debug("Busy?", follower.isBusy());
      panelsTelemetry.update(telemetry);
    }

    follower.breakFollowing();
    telemetry.addLine("Path Complete");
    telemetry.update();


    // keep alive a moment (optional)
    sleep(500);
  }

  // Paths inner class — builds the PathChain(s)
  public static class Paths {

    public PathChain Path1;

    public Paths(Follower follower) {
      // Build the bezier line from the start pose to the desired end pose
      Path1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(25.540, 129.718),   // start point (x, y) - heading already set on follower start
            new Pose(42.175, 114.427)    // end point (x, y)
          )
        )
        .setLinearHeadingInterpolation(Math.toRadians(-43), Math.toRadians(180))
        .build();
    }
  }
}
