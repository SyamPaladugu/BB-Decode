//package org.firstinspires.ftc.teamcode;
//
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import java.util.List;
//
//public class TrackAprilTag  {
//
//    private static final int DESIRED_TAG_ID = 20; // 20 for blue, 24 for red
//
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
//    private boolean targetFound = false;
//    private AprilTagDetection detectedTag = null;// Used to hold the data for a detected AprilTag
//    HardwareMap hardwareMap;
//
//
//   // public TrackAprilTag(HardwareMap){
//        //setManualExposure(6, 250);
//    }
//
//    public boolean seenAprilTag(){
//        return true;
//    }
//
//    public void initApriltag() {
//        aprilTag = new AprilTagProcessor.Builder()
//                // add stuff here
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .build();
//    }
//
//    public void updateDetections() {
//        targetFound = false;
//        detectedTag = null;
//
//        // Step through the list of detected tags and look for a matching tag
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            // Look to see if we have size info on this tag.
//            if (detection.metadata != null) {
//                //  Check to see if we want to track towards this tag.
//                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                    // Yes, we want to use this tag.
//                    targetFound = true;
//                    detectedTag = detection;
//                    break;  // don't look any further.
//
//                }
//            }
//        }
//    }
//
//    // TODO: better name
//    public double[] stuff() {
//        double range = detectedTag.ftcPose.range;
//        double yaw = detectedTag.ftcPose.pitch;
//        double heading = detectedTag.ftcPose.bearing;
//
//        return new double[] {range, yaw, heading};
//    }
//
//
//
//
//
//
//}
