package org.firstinspires.ftc.teamcode.opMode.teleOp.New;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import java.util.List;
//
//@TeleOp(name = "Limelight AprilTag Detection", group = " ")
//public class LimelightAprilTagDetection extends LinearOpMode {
//
//    private Limelight3A limelight;
//
//    @Override
//    public void runOpMode() {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        limelight.pipelineSwitch(0);
//
//        limelight.start();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            LLResult result = limelight.getLatestResult();
//
//            if (result != null && result.isValid()) {
//                telemetry.addData("Target Found", "Yes");
//
//                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//
//                if (fiducialResults != null && !fiducialResults.isEmpty()) {
//                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                        telemetry.addData("Fiducial ID", fr.getFiducialId());
//                        telemetry.addData("Family", fr.getFamily());
//                        telemetry.addData("X", "%.2f", fr.getTargetXDegrees());
//                        telemetry.addData("Y", "%.2f", fr.getTargetYDegrees());
//                        telemetry.addData("Area", "%.2f", fr.getTargetArea());
//                    }
//                }
//            } else {
//                telemetry.addData("Target Found", "No");
//            }
//
//            telemetry.update();
//        }
//
//        limelight.stop();
//    }
//}

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@TeleOp(name = "Lime", group = " ")
@Disabled
public class LimelightAprilTagDetection extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
    }
}