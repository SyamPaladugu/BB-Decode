package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class OdoAprilTagAlignment implements Subsystem {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private MecanumDrive drive;
    private Pose2d goalPose;

    public static double kP = -0.03;
    public static int targetTagID = 20;
    public static double angleTolerance = 0.01;
    public static double maxPower = 1;
    public static double headingToleranceDeg = 2.0;

    private enum AlignmentState {
        IDLE,
        TURNING_TO_GOAL,
        VISUAL_ALIGNMENT,
        ALIGNED
    }

    private AlignmentState state = AlignmentState.IDLE;
    private boolean alignmentActive = false;
    private boolean isAligned = false;

    Telemetry telemetry;

    public OdoAprilTagAlignment(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive) {
        this.telemetry = telemetry;
        this.drive = drive;

        leftFront = drive.leftFront;
        rightFront = drive.rightFront;
        leftBack = drive.leftBack;
        rightBack = drive.rightBack;

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
    }

    @Override
    public void init() {
        telemetry.addData("AprilTag Alignment", "initialized");
        telemetry.addData("Target Tag ID", targetTagID);
        telemetry.update();
    }


    public void setGoalPose(Pose2d goalPose) {
        this.goalPose = goalPose;
    }

    @Override
    public void update() {
        drive.updatePoseEstimate();

        if (alignmentActive) {
            performStateMachine();
        }

        telemetry.addLine("=== AprilTag Alignment ===");
        telemetry.addData("State", state);
        telemetry.addData("Mode", alignmentActive ? "AUTO ALIGN" : "MANUAL");

        if (alignmentActive) {
            Pose2d currentPose = drive.localizer.getPose();
            telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, H: %.2f°",
                    currentPose.position.x, currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble()));

            if (goalPose != null) {
                double targetHeading = getHeadingToGoal(currentPose);
                double headingError = getHeadingError(currentPose.heading.toDouble(), targetHeading);
                telemetry.addData("Target Heading", "%.2f°", Math.toDegrees(targetHeading));
                telemetry.addData("Heading Error", "%.2f°", Math.toDegrees(headingError));
            }

            if (isTagDetected()) {
                telemetry.addData("Tag Detected", "ID %d", targetTagID);
                telemetry.addData("Aligned", isAligned ? "✓ YES" : "✗ NO");
            } else {
                telemetry.addLine("Target tag not visible");
            }

            telemetry.addLine("--- Tuning Values ---");
            telemetry.addData("kP", kP);
            telemetry.addData("Target Tag ID", targetTagID);
            telemetry.addData("Angle Tolerance", angleTolerance);
            telemetry.addData("Heading Tolerance", headingToleranceDeg);
            telemetry.addData("Max Power", maxPower);
        }
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.xWasPressed()) {
            alignmentActive = true;
            isAligned = false;

            if (isTagDetected()) {
                state = AlignmentState.VISUAL_ALIGNMENT;
            } else {
                state = AlignmentState.TURNING_TO_GOAL;
            }
        }

        if (gp1.xWasReleased()) {
            alignmentActive = false;
            isAligned = false;
            state = AlignmentState.IDLE;
            stopMotors();
        }

        telemetry.addData("Aligned Properly", isAligned && alignmentActive);
    }

    public boolean isAlignmentActive() {
        return alignmentActive;
    }

    private void performStateMachine() {
        switch (state) {
            case TURNING_TO_GOAL:
                handleTurningToGoal();
                break;
            case VISUAL_ALIGNMENT:
                handleVisualAlignment();
                break;
            case ALIGNED:
                handleAligned();
                break;
            case IDLE:
                break;
        }
    }

    private void handleTurningToGoal() {
        if (isTagDetected()) {
            state = AlignmentState.VISUAL_ALIGNMENT;
            return;
        }

        Pose2d currentPose = drive.localizer.getPose();
        double targetHeading = getHeadingToGoal(currentPose);
        double currentHeading = currentPose.heading.toDouble();
        double headingError = getHeadingError(currentHeading, targetHeading);
        double errorDegrees = Math.toDegrees(headingError);

        if (Math.abs(errorDegrees) < headingToleranceDeg) {
            state = AlignmentState.VISUAL_ALIGNMENT;
            return;
        }

        double turnPower = kP * errorDegrees;
        turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

        leftFront.setPower(-turnPower);
        leftBack.setPower(-turnPower);
        rightFront.setPower(turnPower);
        rightBack.setPower(turnPower);
    }

    private void handleVisualAlignment() {
        if (!isTagDetected()) {
            state = AlignmentState.TURNING_TO_GOAL;
            isAligned = false;
            return;
        }

        performAlignment();

        if (isAligned) {
            state = AlignmentState.ALIGNED;
        }
    }

    private void handleAligned() {
        if (!isTagDetected()) {
            state = AlignmentState.TURNING_TO_GOAL;
            isAligned = false;
            return;
        }

        performAlignment();

        if (!isAligned) {
            state = AlignmentState.VISUAL_ALIGNMENT;
        }
    }

    private void performAlignment() {
        AprilTagDetection tag = getTagByID(targetTagID);

        if (tag == null) {
            stopMotors();
            isAligned = false;
            return;
        }

        double x = tag.ftcPose.x;
        double y = tag.ftcPose.y;
        double angleToTag = Math.atan2(x, y);
        double error = Math.toDegrees(angleToTag);

        double rotationPower = kP * error;

        // Clamp power
        rotationPower = Math.max(-maxPower, Math.min(maxPower, rotationPower));

        // Apply rotation
        leftFront.setPower(-rotationPower);
        leftBack.setPower(-rotationPower);
        rightFront.setPower(rotationPower);
        rightBack.setPower(rotationPower);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("power", rotationPower);
        telemetry.addData("error angle", angleToTag);

        if (Math.abs(error) < angleTolerance) {
            isAligned = true;
        } else {
            isAligned = false;
        }
    }

    private double getHeadingToGoal(Pose2d currentPose) {
        if (goalPose == null) return 0;

        double dx = goalPose.position.x - currentPose.position.x;
        double dy = goalPose.position.y - currentPose.position.y;

        return Math.atan2(dy, dx);
    }

    private double getHeadingError(double currentHeading, double targetHeading) {
        double error = targetHeading - currentHeading;

        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        return error;
    }

    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public double getCurrentAngle() {
        AprilTagDetection tag = getTagByID(targetTagID);
        return tag != null ? tag.ftcPose.yaw : 0.0;
    }

    public boolean isTagDetected() {
        return getTagByID(targetTagID) != null;
    }

    public boolean isAligned() {
        return isAligned;
    }

    private AprilTagDetection getTagByID(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void close() {
        visionPortal.close();
    }
}