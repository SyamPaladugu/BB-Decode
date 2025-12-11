package org.firstinspires.ftc.teamcode.opMode.auto.New;

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
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.New.CustomAdaptiveIntake;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.New.Shooter2;
import org.firstinspires.ftc.teamcode.subsystem.New.Turret;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Kicker;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;

@Config
@Autonomous(name = "BlueFar", group = "Autonomous")
public class RedFar extends LinearOpMode {

    Pose2d initialPose = new Pose2d(-49.5, 49.5, Math.toRadians(305));
    MecanumDrive follower;

    Shooter2 shooter;
    CustomAdaptiveIntake customAdaptiveIntake;

    Turret turret;
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
        shooter = new Shooter2(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        customAdaptiveIntake = new CustomAdaptiveIntake(hardwareMap, telemetry);


        build_paths();



        waitForStart();
        if (isStopRequested()) return;
        time.startTime();
        time.reset();

        while (opModeIsActive()) {
            follower.updatePoseEstimate();
            Pose2d currentPose = follower.localizer.getPose();

            turret.updatePose(currentPose.position, Math.toDegrees(currentPose.heading.toDouble()));

            update();


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
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-23.3,29,Math.toRadians(70)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12,48.1,Math.toRadians(77)),Math.toRadians(100))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos1 = intakeSpike1.fresh()
                .setTangent(Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(-23.6,23.3,Math.toRadians(135)),Math.toRadians(220))
                .splineToSplineHeading(new Pose2d(-23.6,23.3,Math.toRadians(90)),Math.toRadians(220))
                .waitSeconds(5);
        TrajectoryActionBuilder intakeSpike2 = shootPos1.fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(7,27.8,Math.toRadians(70)), Math.toRadians(60))
                .splineToSplineHeading(new Pose2d(12,44,Math.toRadians(88)),Math.toRadians(90))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos2 = intakeSpike2.fresh()
                .strafeTo(new Vector2d(-23.3,23.1))
                .waitSeconds(3.5);
        TrajectoryActionBuilder intakeSpike3 = shootPos2.fresh()
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(33,28.6,Math.toRadians(70)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(37, 46,Math.toRadians(88)), Math.toRadians(280))
                .waitSeconds(0.3);
        TrajectoryActionBuilder shootPos3 = intakeSpike3.fresh()
                .strafeTo(new Vector2d(-23.8,23.8))
                .waitSeconds(3.5);
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
                customAdaptiveIntake.pivIntake();
                customAdaptiveIntake.autoIntake();
                turret.setAligning(false);

                if (!currentAction) {
                    state = AutoStates.SHOOT1;
                    time.reset();
                }
                break;
            case SHOOT1:
                currentAction = shoot1.run(packet);
                customAdaptiveIntake.autoIntakeOff();
                if (time.seconds() >= 1.4){
                    turret.setAligning(true);
                    shooter.spinUpClose();
                }
                if (time.seconds() >= 3){
                    customAdaptiveIntake.pivSendBalls();
                }
                if (time.seconds() >= 3.7){
                    customAdaptiveIntake.autoIntakeOff();
                }

                if (!currentAction) {
                    state = AutoStates.INTAKE2;
                    time.reset();

                }
                break;
            case INTAKE2:
                currentAction = intake2.run(packet);
                shooter.stop();
                customAdaptiveIntake.pivIntake();
                customAdaptiveIntake.autoIntake();
                turret.setAligning(false);

                if (!currentAction) {
                    state = AutoStates.SHOOT2;
                    time.reset();
                }
                break;

            case SHOOT2:
                currentAction = shoot2.run(packet);
                customAdaptiveIntake.autoIntakeOff();
                if (time.seconds() >= 1.84){
                    turret.setAligning(true);
                    shooter.spinUpClose();
                }

                if (time.seconds() >= 3){
                    customAdaptiveIntake.pivSendBalls();
                }

                if (!currentAction) {
                    state = AutoStates.INTAKE3;
                    time.reset();
                }
                break;


            case INTAKE3:
                currentAction = intake3.run(packet);
                shooter.stop();
                customAdaptiveIntake.pivIntake();
                customAdaptiveIntake.autoIntake();
                turret.setAligning(false);

                if (!currentAction){
                    state = AutoStates.SHOOT3;
                    time.reset();
                }
                break;


            case SHOOT3:
                currentAction = shoot3.run(packet);
                if (time.seconds() >= 2.12){
                    turret.setAligning(true);
                    shooter.spinUpClose();
                }
                if (time.seconds() >= 3.5){
                    customAdaptiveIntake.pivSendBalls();
                }

                if (!currentAction) {
                    state = AutoStates.PARK;
                    time.reset();
                }
                break;

            case PARK:
                currentAction = park.run(packet);
                shooter.stop();
                turret.setAligning(false);
                customAdaptiveIntake.pivIntake();

                if (!currentAction){
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