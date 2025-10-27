package org.firstinspires.ftc.teamcode.opMode.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTag Center-Track (No Tag Size, Any 36h11)", group=" ")
public class AprilTagTrack extends LinearOpMode {

    private static final int IMG_WIDTH = 1280;
    private static final double HFOV_DEG = 70.0;
    private static final double ROBOT_CENTER_OFFSET_DEG = 0.0;

    private static final double CAM_X_IN = 4.0;
    private static final double CAM_Z_IN = 6.0;
    private static final double CAM_YAW_DEG = 0.0;

    private static final double MAX_ROT_POWER = 0.5;
    private final PID headingPid = new PID(0.010, 0.0002, 0.002);

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private AprilTagProcessor tagProc;
    private VisionPortal portal;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "LFM");
        rightFront = hardwareMap.get(DcMotorEx.class, "RFM,");
        leftBack   = hardwareMap.get(DcMotorEx.class, "LBM");
        rightBack  = hardwareMap.get(DcMotorEx.class, "RBM");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor dcMotor : new DcMotor[]{leftFront,rightFront,leftBack,rightBack}) {
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        try {
            tagProc = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();
            telemetry.addLine("Vision ready (camera starts on PLAY).");
        } catch (Exception e) {
            tagProc = null;
            telemetry.addLine("Vision init failed; driving only.");
        }
        telemetry.update();

        waitForStart();

        ensureVisionStreaming();

        long lastT = System.nanoTime();
        headingPid.reset();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastT) / 1e9;
            lastT = now;

            double forwardCmd = -gamepad1.left_stick_y * 0.7;
            double strafeCmd  =  gamepad1.left_stick_x * 0.7;

            Double headingErrDeg = getHeadingErrorDegAnyMode();

            double turnCmd;
            if (headingErrDeg != null) {
                double pidOut = headingPid.calculate(headingErrDeg, dt);
                turnCmd = clamp(pidOut, -MAX_ROT_POWER, MAX_ROT_POWER);
                telemetry.addData("Tag", "seen");
                telemetry.addData("headingErr", "%.2f deg", headingErrDeg);
            } else {
                telemetry.addData("Tag", "not seen");
                headingPid.reset();
                turnCmd = gamepad1.right_stick_x * 0.6;
            }

            telemetry.addData("CameraState", (portal == null) ? "null" : portal.getCameraState());
            setDrive(forwardCmd, strafeCmd, turnCmd);
            telemetry.update();

            if (gamepad1.a && portal != null) portal.stopStreaming();
            if (gamepad1.b && portal != null) portal.resumeStreaming();
        }

        setDrive(0,0,0);
        if (portal != null) portal.close();
    }

    private void ensureVisionStreaming() {
        if (tagProc == null) return;
        if (portal == null) {
            WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
            portal = new VisionPortal.Builder()
                    .setCamera(cam)
                    .setCameraResolution(new Size(IMG_WIDTH, 720))
                    .addProcessor(tagProc)
                    .build();
        } else {
            portal.resumeStreaming();
        }
        long deadline = System.currentTimeMillis() + 1000;
        while (!isStopRequested()
                && System.currentTimeMillis() < deadline
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
    }

    private Double getHeadingErrorDegAnyMode() {
        if (tagProc == null || portal == null
                || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return null;

        List<AprilTagDetection> dets = tagProc.getDetections();
        int n = (dets == null) ? 0 : dets.size();
        telemetry.addData("#detections", n);
        if (n == 0) return null;

        AprilTagDetection best = null;
        double bestDx = Double.POSITIVE_INFINITY;
        for (AprilTagDetection d : dets) {
            double dx = Math.abs(d.center.x - IMG_WIDTH * 0.5);
            if (dx < bestDx) { bestDx = dx; best = d; }
            telemetry.addData("id", d.id);
        }
        if (best == null) return null;

        if (best.ftcPose != null) {
            double tx_right   = -best.ftcPose.y;
            double tz_forward =  best.ftcPose.x;
            telemetry.addData("mode", "pose");
            return robotCenterHeadingErrorDeg(tx_right, tz_forward, CAM_X_IN, CAM_Z_IN, CAM_YAW_DEG);
        }

        telemetry.addData("mode", "pixel");
        double pxErr = (best.center.x - IMG_WIDTH * 0.5);
        double norm  = pxErr / (IMG_WIDTH * 0.5);
        double angleDeg = norm * (HFOV_DEG * 0.5);
        return angleDeg + ROBOT_CENTER_OFFSET_DEG;
    }

    private static double robotCenterHeadingErrorDeg(
            double txCamRight, double tzCamForward,
            double camX_in, double camZ_in, double camYaw_deg) {

        double yaw = Math.toRadians(camYaw_deg);
        double cos = Math.cos(yaw), sin = Math.sin(yaw);

        double vx_r_from_cam =  cos * txCamRight + sin * tzCamForward;
        double vz_r_from_cam = -sin * txCamRight + cos * tzCamForward;

        double vx_r = camX_in + vx_r_from_cam;
        double vz_r = camZ_in + vz_r_from_cam;

        return Math.toDegrees(Math.atan2(vx_r, vz_r));
    }

    private void setDrive(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        lf /= max; rf /= max; lb /= max; rb /= max;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    private static class PID {
        double kP, kI, kD, integ = 0, prevErr = 0, integLimit = 1.0;
        PID(double p, double i, double d){kP=p; kI=i; kD=d;}
        void reset(){integ=0; prevErr=0;}
        double calculate(double error, double dt){
            integ += error * dt;
            if (integ >  integLimit) integ =  integLimit;
            if (integ < -integLimit) integ = -integLimit;
            double deriv = (error - prevErr) / Math.max(dt, 1e-6);
            prevErr = error;
            return kP*error + kI*integ + kD*deriv;
        }
    }
    private static double clamp(double v,double lo,double hi){ return Math.max(lo, Math.min(hi, v)); }
}