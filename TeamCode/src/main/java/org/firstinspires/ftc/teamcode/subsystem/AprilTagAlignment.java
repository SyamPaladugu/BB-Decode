package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AprilTagAlignment implements Subsystem {

    // Motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public static double kP = -0.03;
    public static int targetTagID = 20;
    public static double angleTolerance = 0.01;
    public static double maxPower = 1;

    // State
    private boolean alignmentActive = false;
    private boolean isAligned = false;

    Telemetry telemetry;

    public AprilTagAlignment(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "LFM");
        rightFront = hardwareMap.get(DcMotor.class, "RFM");
        leftBack = hardwareMap.get(DcMotor.class, "LBM");
        rightBack = hardwareMap.get(DcMotor.class, "RBM");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Initialize vision
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

    @Override
    public void update() {
        if (alignmentActive) {
            performAlignment();
        }

        // Add telemetry
        telemetry.addLine("=== AprilTag Alignment ===");
        telemetry.addData("Mode", alignmentActive ? "AUTO ALIGN" : "MANUAL");

        if (alignmentActive) {
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
            telemetry.addData("Max Power", maxPower);
        }
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.xWasPressed()) {
            alignmentActive = true;
        }
        if(gp1.xWasReleased()) {
            alignmentActive = false;
            isAligned = false;
            stopMotors();
        }

        telemetry.addData("Aligned Properly", isAligned && alignmentActive);
    }

    public boolean isAlignmentActive() {
        return alignmentActive;
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
        telemetry.addData("y",y);
        telemetry.addData("power", rotationPower);
        telemetry.addData("error angle", (angleToTag));
        if (Math.abs(error) < angleTolerance) {
            isAligned = true;
        } else {
            isAligned = false;
        }
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