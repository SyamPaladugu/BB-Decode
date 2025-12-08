package org.firstinspires.ftc.teamcode.opMode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(name = "Turret Test", group = "test")
public class TurretTest extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(-49.9, -49.7, Math.toRadians(55));

    private Drivetrain drivetrain;
    private Turret turret;
    private MecanumDrive follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Road Runner drive (for odometry only)
        follower = new MecanumDrive(hardwareMap, START_POSE);

        // Initialize subsystems
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);

        drivetrain.init();
        turret.init();

        telemetry.addLine("===== TURRET TEST TELEOP =====");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("- Left stick: Drive forward/back & strafe");
        telemetry.addLine("- Right stick X: Rotate");
        telemetry.addLine("- RIGHT BUMPER: Align turret to goal");
        telemetry.addLine();
        telemetry.addLine("Adjust goal position in FTC Dashboard:");
        telemetry.addLine("- GOAL_X and GOAL_Y");
        telemetry.addLine();
        telemetry.addData("Starting Position", "X: %.1f, Y: %.1f", START_POSE.position.x, START_POSE.position.y);
        telemetry.addData("Starting Heading", "%.1f degrees", Math.toDegrees(START_POSE.heading.toDouble()));
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Road Runner odometry (for position tracking only)
            follower.updatePoseEstimate();

            // Get current pose from Road Runner
            Pose2d currentPose = follower.localizer.getPose();
            Vector2d currentPos = currentPose.position;
            double currentHeading = Math.toDegrees(currentPose.heading.toDouble());

            // Update turret with position and heading
            turret.updatePose(currentPos, currentHeading);

            // Update drivetrain controls (uses your subsystem for driving)
            drivetrain.updateCtrls(gamepad1, gamepad2);

            // Update turret controls - turret handles the logic internally
            turret.updateCtrls(gamepad1, gamepad2);
            turret.update();

            // Additional telemetry for testing
            telemetry.addLine("===== ROBOT STATE =====");
            telemetry.addData("Position", "X: %.2f, Y: %.2f", currentPos.x, currentPos.y);
            telemetry.addData("Heading", "%.2f degrees", currentHeading);
            telemetry.addLine();
            telemetry.addLine("===== TURRET STATE =====");
            telemetry.addData("Distance to Goal", "%.2f inches", turret.getDistanceToGoal());
            telemetry.addData("Turret Angle", "%.2f degrees", turret.getCurrentTurretAngle());
            telemetry.addData("Aligning", turret.isAligning() ? "YES" : "NO");
            telemetry.addLine();
            telemetry.addData("Goal Position", "X: %.1f, Y: %.1f", Turret.GOAL_X, Turret.GOAL_Y);
            telemetry.update();
        }
    }
}