package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class  ApriltagRange {

    private AprilTagProcessor aprilTagProcessor;

    /**
     * Constructor for ApriltagRange
     */
    public ApriltagRange( ) {


    }


    public void init(HardwareMap hardwareMap) {
        // In your OpMode initialization
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();


    }
    /**
     * Gets the range (distance) to the AprilTag
     * @param tagId The ID of the AprilTag you want to find
     * @return The range in inches, or -1 if tag not detected
     */
    public double getRange(int tagId) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == tagId) {
                return detection.ftcPose.range;
            }
        }

        // Return -1 if tag not found
        return -1.0;
    }

    /**
     * Gets the range to the first detected AprilTag
     * @return The range in inches, or -1 if no tags detected
     */
    public double getRange() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.size() > 0) {
            return detections.get(0).ftcPose.range;
        }

        return -1.0;
    }

    /**
     * Gets the bearing (angle) to the AprilTag
     * @param tagId The ID of the AprilTag
     * @return The bearing in degrees, or 0 if not detected
     */
    public double getBearing(int tagId) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == tagId) {
                return detection.ftcPose.bearing;
            }
        }

        return 0.0;
    }
}