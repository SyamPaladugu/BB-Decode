package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

import java.util.List;

@Config
public class LimelightTurretTest implements Subsystem {

    private Servo turretServo1;
    private Servo turretServo2;
    private Limelight3A limelight;

    public static double degreesPerServo = 270;
    public static int targetTagID = 20;
    public static double angleTolerance = 1.0;
    public static double minServoPosition = 0.0;
    public static double maxServoPosition = 1.0;
    public static int pipeline = 0;

    private boolean alignmentActive = false;
    private boolean isAligned = false;
    private double currentServoPosition = 0.5;

    private Telemetry telemetry;

    public LimelightTurretTest(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        turretServo1 = hardwareMap.get(Servo.class, "turret1");
        turretServo2 = hardwareMap.get(Servo.class, "turret2");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();


        currentServoPosition = turretServo1.getPosition();
    }

    @Override
    public void init() {
        telemetry.addData("Limelight Turret Alignment", "Initialized");
        telemetry.addData("Target Tag ID", targetTagID);
        telemetry.update();
    }

    @Override
    public void update() {
        if (alignmentActive) {
            performAlignment();
        }

        telemetry.addLine("=== Limelight Turret Alignment ===");
        telemetry.addData("Mode", alignmentActive ? "AUTO ALIGN" : "MANUAL");
        telemetry.addData("Servo Position", String.format("%.3f", currentServoPosition));

        if (alignmentActive) {
            if (isTagDetected()) {
                telemetry.addData("Tag Detected", "ID %d", targetTagID);
                telemetry.addData("Aligned", isAligned ? "✓ YES" : "✗ NO");
            } else {
                telemetry.addLine("Target tag not visible");
            }

            telemetry.addLine("--- Tuning Values ---");
            telemetry.addData("Degrees Per Unit", degreesPerServo);
            telemetry.addData("Target Tag ID", targetTagID);
            telemetry.addData("Angle Tolerance", angleTolerance);
            telemetry.addData("Servo Range", String.format("%.2f - %.2f", minServoPosition, maxServoPosition));
        }
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.xWasPressed()) {
            alignmentActive = true;
        }
        if (gp1.xWasReleased()) {
            alignmentActive = false;
            isAligned = false;
        }

        telemetry.addData("Aligned Properly", isAligned && alignmentActive);
    }

    public boolean isAlignmentActive() {
        return alignmentActive;
    }

    private void performAlignment() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            isAligned = false;
            return;
        }

        LLResultTypes.FiducialResult targetTag = getTagByID(result, targetTagID);

        if (targetTag == null) {
            isAligned = false;
            return;
        }

        double x = targetTag.getTargetXDegrees();
        double y = targetTag.getTargetYDegrees();
        double angleToTag = Math.atan2(x, y);
        double angleDegrees = Math.toDegrees(angleToTag);

        currentServoPosition = angleDegrees / degreesPerServo;
        currentServoPosition = Math.max(minServoPosition, Math.min(maxServoPosition, currentServoPosition));

        turretServo1.setPosition(currentServoPosition);
        turretServo2.setPosition(currentServoPosition);

        telemetry.addData("tx", x);
        telemetry.addData("ty", y);
        telemetry.addData("angle to tag", angleDegrees);
        telemetry.addData("servo position", currentServoPosition);

        if (Math.abs(angleDegrees) < angleTolerance) {
            isAligned = true;
        } else {
            isAligned = false;
        }
    }

    public boolean isTagDetected() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }
        return getTagByID(result, targetTagID) != null;
    }

    public boolean isAligned() {
        return isAligned;
    }

    private LLResultTypes.FiducialResult getTagByID(LLResult result, int id) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == id) {
                return fiducial;
            }
        }
        return null;
    }

    public Pose3D getBotpose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    public void setServoPosition(double position) {
        currentServoPosition = Math.max(minServoPosition, Math.min(maxServoPosition, position));
        turretServo1.setPosition(currentServoPosition);
        turretServo2.setPosition(currentServoPosition);
    }

    public double getServoPosition() {
        return currentServoPosition;
    }

    public void close() {
        limelight.stop();
    }
}