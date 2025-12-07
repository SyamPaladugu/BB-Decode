package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.OdoAprilTagAlignment;

@TeleOp(name = "OdoAprilTagAlignment", group = " ")
public class OdoAprilTagAlignmentTele extends LinearOpMode {

    private static final Pose2d STARTING_POSE = new Pose2d(-49.9, -49.7, Math.toRadians(55));
    private static final Pose2d GOAL_POSE = new Pose2d(-62.7, -56.6, Math.toRadians(225));

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose;
        if (PoseStorage.poseFromAuto) {
            startPose = PoseStorage.currentPose;
            telemetry.addLine("Starting from AUTO final position");
        } else {
            startPose = STARTING_POSE;
            telemetry.addLine("Starting from DEFAULT position");
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);

        OdoAprilTagAlignment odoAprilTagAlignment = new OdoAprilTagAlignment(hardwareMap, telemetry, drive);

        odoAprilTagAlignment.setGoalPose(GOAL_POSE);

        odoAprilTagAlignment.init();
        drivetrain.init();

        telemetry.addLine("Ready to start!");
        telemetry.addData("Starting Pose", startPose.toString());
        telemetry.addData("Goal Pose", GOAL_POSE.toString());
        telemetry.addData("Pose Source", PoseStorage.poseFromAuto ? "AUTO" : "DEFAULT");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  X Button (hold) - Align to AprilTag");
        telemetry.update();

        waitForStart();

        PoseStorage.poseFromAuto = false;

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            odoAprilTagAlignment.updateCtrls(gamepad1, gamepad2);
            drivetrain.updateCtrls(gamepad1, gamepad2);
            odoAprilTagAlignment.update();

            Pose2d currentPose = drive.localizer.getPose();
            telemetry.addLine("=== Robot Position ===");
            telemetry.addData("X", "%.2f", currentPose.position.x);
            telemetry.addData("Y", "%.2f", currentPose.position.y);
            telemetry.addData("Heading", "%.2f°", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addLine();

            telemetry.update();
        }

        odoAprilTagAlignment.close();
    }
}