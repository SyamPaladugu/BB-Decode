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

    public DcMotorEx leftFront, rightFront, leftBack, rightBack;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public static double kP = 0.03;
    public static int targetTagID = 20;
    public static double angleTolerance = 2.0;
    public static double maxPower = 0.5;

    public boolean alignmentActive = false;
    public boolean isAligned = false;

    Telemetry telemetry;

    public AprilTagAlignment(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotorEx.class, "LFM");
        rightFront = hardwareMap.get(DcMotorEx.class, "RFM");
        leftBack = hardwareMap.get(DcMotorEx.class, "LBM");
        rightBack = hardwareMap.get(DcMotorEx.class, "RBM");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

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

        telemetry.addLine("=== AprilTag Alignment ===");
        telemetry.addData("Mode", alignmentActive ? "AUTO ALIGN" : "MANUAL");

        if (isTagDetected()) {
            telemetry.addData("Tag Detected", "ID %d", targetTagID);
            telemetry.addData("Yaw Angle", "%.1f°", getCurrentAngle());
            telemetry.addData("Error", "%.1f°", getCurrentAngle());
            telemetry.addData("Aligned", isAligned ? "✓ YES" : "✗ NO");
        } else {
            telemetry.addLine("Target tag not visible");
        }

        // Display tuning values
        telemetry.addLine("--- Tuning Values ---");
        telemetry.addData("kP", kP);
        telemetry.addData("Target Tag ID", targetTagID);
        telemetry.addData("Angle Tolerance", angleTolerance);
        telemetry.addData("Max Power", maxPower);
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.a) {
            alignmentActive = true;
        } else {
            alignmentActive = false;
            isAligned = false;
            stopMotors();
        }

        if (isAligned && alignmentActive) {
            gp1.rumble(500);
        }
    }

    public boolean isAlignmentActive() {
        return alignmentActive;
    }

    public void cancelAlignment() {
        alignmentActive = false;
        isAligned = false;
        stopMotors();
    }

    private void performAlignment() {
        AprilTagDetection tag = getTagByID(targetTagID);

        if (tag == null) {
            stopMotors();
            isAligned = false;
            return;
        }

        double currentAngle = tag.ftcPose.yaw;
        double error = -currentAngle;

        if (Math.abs(error) < angleTolerance) {
            stopMotors();
            isAligned = true;
            alignmentActive = false;  // Stop alignment once achieved
            return;
        }

        double rotationPower = kP * error;

        rotationPower = Math.max(-maxPower, Math.min(maxPower, rotationPower));

        leftFront.setPower(-rotationPower);
        leftBack.setPower(-rotationPower);
        rightFront.setPower(rotationPower);
        rightBack.setPower(rotationPower);

        isAligned = false;
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