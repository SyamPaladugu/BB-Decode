package org.firstinspires.ftc.teamcode.opMode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Kicker;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;

@Config
@Autonomous(name = "BlueFar", group = "Autonomous")
public class BlueSideFar extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-49.9, -49.7, Math.toRadians(55));

    MecanumDrive follower;
    Outtake outtake;

    Kicker kicker;
    Intake intake;
    Transfer transfer;
    Action intake1, shoot1, intake2, shoot2, intake3, shoot3, park;
    boolean currentAction;
    enum AutoStates {
        START,
        INTAKE1,
        SHOOT1,
        INTAKE2,
        SHOOT2,
        INTAKE3,
        SHOOT3,
        PARK,
        END
    }

     AutoStates state = AutoStates.START;


     ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        follower = new MecanumDrive(hardwareMap, initialPose);
        outtake = new Outtake(hardwareMap, telemetry);
        kicker = new Kicker(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        build_paths();



        waitForStart();
        if (isStopRequested()) return;
        time.startTime();
        time.reset();

        while (opModeIsActive()) {
            follower.updatePoseEstimate();

            update();

            Pose2d currentPose = follower.localizer.getPose();
            telemetry.addData("State", state);
            telemetry.addData("X", currentPose.position.x);
            telemetry.addData("Y", currentPose.position.y);
            telemetry.addData("Heading", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Current State", state.name());
            telemetry.addData("State Time", "%.2f sec", time.seconds());
            telemetry.update();
        }


    }

    public void build_paths() {
        TrajectoryActionBuilder intakeSpike1 = follower.actionBuilder(initialPose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-23.3,-27,Math.toRadians(270)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-15,-34.5,Math.toRadians(290)),Math.toRadians(290))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos1 = intakeSpike1.fresh()
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-21.9,-22,Math.toRadians(225)),Math.toRadians(140))
                .waitSeconds(5);
        TrajectoryActionBuilder intakeSpike2 = shootPos1.fresh()
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(5.3,-27.8,Math.toRadians(290)), Math.toRadians(310))
                .splineToSplineHeading(new Pose2d(11.5,-50,Math.toRadians(270)),Math.toRadians(270))
                .waitSeconds(0.4);
        TrajectoryActionBuilder shootPos2 = intakeSpike2.fresh()
                .strafeToLinearHeading(new Vector2d(-22.9,-22.7), Math.toRadians(225))
                .waitSeconds(3.2);
        TrajectoryActionBuilder intakeSpike3 = shootPos2.fresh()
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(24.1,-28.6,Math.toRadians(290)), Math.toRadians(330))
                .splineToLinearHeading(new Pose2d(34.4, -49,Math.toRadians(270)), Math.toRadians(280));
        TrajectoryActionBuilder shootPos3 = intakeSpike3.fresh()
                .strafeToLinearHeading(new Vector2d(-23.8,-23.8), Math.toRadians(225))
                .waitSeconds(4.7);
        TrajectoryActionBuilder parkPos = shootPos3.fresh()
                .strafeToLinearHeading(new Vector2d(0,-48.8),Math.toRadians(180));

        intake1 = intakeSpike1.build();
        shoot1 = shootPos1.build();
        intake2 = intakeSpike2.build();
        shoot2 = shootPos2.build();
        intake3 = intakeSpike3.build();
        shoot3 = shootPos3.build();
        park = parkPos.build();


    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (state) {
            case START:
                time.reset();
                state = AutoStates.INTAKE1;

            case INTAKE1:
                currentAction = intake1.run(packet);
                intake.intake.setPower(-1);
                transfer.transfer.setPower(-1);
                kicker.kicker.setPosition(0.3);

                if (time.seconds() >= 1){
                    transfer.transfer.setPower(0);
                }

                if (!currentAction) {
                    state = AutoStates.SHOOT1;
                    time.reset();
                }
                break;
            case SHOOT1:
                currentAction = shoot1.run(packet);


                if (time.seconds() >= 1.4){
                    intake.intake.setPower(0);
                    outtake.outtake.setPower(1);
                }
                if (time.seconds()>=3.7){
                    intake.intake.setPower(-0.7);

                }

                if (time.seconds() >= 4.5){
                    kicker.kicker.setPosition(0);
                    transfer.transfer.setPower(-1);
                }
                if (time.seconds() >= 4.85){
                    kicker.kicker.setPosition(0.3);
                }
                if (time.seconds() >= 5.1){
                    kicker.kicker.setPosition(0);
                }

                if (time.seconds() >= 5.2) {
                    outtake.outtake.setPower(0);
                    kicker.kicker.setPosition(0.3);
                }
                if (!currentAction) {
                    state = AutoStates.INTAKE2;
                    time.reset();

                }
                break;
            case INTAKE2:
                currentAction = intake2.run(packet);
                intake.intake.setPower(-1);
                transfer.transfer.setPower(-1);

                if (!currentAction) {
                    state = AutoStates.SHOOT2;
                    time.reset();
                }
                break;

            case SHOOT2:
                currentAction = shoot2.run(packet);

                if (time.seconds() >= 1){
                    intake.intake.setPower(0);
                }
                if (time.seconds()>= 2){
                    outtake.outtake.setPower(1);
                }
                if (time.seconds() >= 3){
                    intake.intake.setPower(-0.7);
                    transfer.transfer.setPower(-1);
                }
                if (time.seconds() >= 3.4) {
                    kicker.kicker.setPosition(0);
                }

                if (time.seconds() >= 4){
                    outtake.outtake.setPower(0);
                    kicker.kicker.setPosition(0.3);
                }
                if (!currentAction) {
                    state = AutoStates.INTAKE3;
                    time.reset();
                }
                break;


            case INTAKE3:
                currentAction = intake3.run(packet);
                intake.intake.setPower(-1);
                transfer.transfer.setPower(-1);

                if (!currentAction){
                    state = AutoStates.SHOOT3;
                    time.reset();
                }
                break;


            case SHOOT3:
                currentAction = shoot3.run(packet);
                if (time.seconds()>=1){
                    intake.intake.setPower(0);
                }

                if (time.seconds() >= 2.25){
                    outtake.outtake.setPower(1);
                }
                if (time.seconds()>=4.3){
                    transfer.transfer.setPower(-1);
                    intake.intake.setPower(-0.7);
                }
                if (time.seconds() >= 5){
                    kicker.kicker.setPosition(0);
                }
                if (time.seconds() >= 5.7) {
                    kicker.kicker.setPosition(0.3);
                }
                if (!currentAction) {
                    state = AutoStates.PARK;
                    time.reset();
                }
                break;

            case PARK:
                currentAction = park.run(packet);
                outtake.outtake.setPower(0);
                if (!currentAction){
                    PoseStorage.currentPose = follower.localizer.getPose();
                    PoseStorage.poseFromAuto = true;
                    state = AutoStates.END;
                    time.reset();
                }

            case END:
                // do nothing
                break;

        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}