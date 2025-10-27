package org.firstinspires.ftc.teamcode.opMode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;

@Autonomous(name = "2ArtifactPP", group = " ")
@Configurable // Panels
public class TwoArtifactPP extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer actionTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(71.809, 8.043),
                                    new Pose(72.191, 45.000),
                                    new Pose(50.362, 93.830)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.362, 93.830),
                                    new Pose(46.915, 83.489),
                                    new Pose(29.298, 84.255)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(29.298, 84.255),
                                    new Pose(46.723, 83.681),
                                    new Pose(50.362, 93.638)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(50.362, 93.638),
                                    new Pose(48.830, 55.532),
                                    new Pose(38.681, 34.085)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}
