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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Kicker;

@Config
@Autonomous(name = "BlueFar", group = "Autonomous")
public class BlueSideFar extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-49.9, -49.7, Math.toRadians(55));

    MecanumDrive follower;
    Outtake outtake;

    Kicker kicker;
    Intake intake;
    Action intake1, shoot1, intake2, shoot2, intake3, shoot3;
    boolean currentAction;

    enum AutoStates {
        INTAKE1,
        SHOOT1,
        INTAKE2,
        SHOOT2,
        INTAKE3,
        SHOOT3,
        END
    }

    AutoStates state;

    public ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new MecanumDrive(hardwareMap, initialPose);

        build_paths();


        waitForStart();
        if (isStopRequested()) return;
        time.reset();

        while (opModeIsActive()) {
            update();
        }


    }

    public void build_paths() {
        TrajectoryActionBuilder intakeSpike1 = follower.actionBuilder(initialPose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-12.2, -27, Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.5)
                .lineToY(-51.1)
                .waitSeconds(0.1)
                .lineToY(-27)
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos1 = intakeSpike1.fresh()
                .splineToLinearHeading(new Pose2d(-21.9, -21.9, Math.toRadians(225)), Math.toRadians(0))
                .waitSeconds(2);
        TrajectoryActionBuilder intakeSpike2 = shootPos1.fresh()
                .strafeToLinearHeading(new Vector2d(10.9, -27.7), Math.toRadians(270))
                .waitSeconds(0.7)
                .strafeTo(new Vector2d(11.5, -51.1))
                .strafeTo(new Vector2d(10.9, -27.7))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos2 = intakeSpike2.fresh()
                .setTangent(-225)
                .splineToLinearHeading(new Pose2d(-21.9, -21.9, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(2);
        TrajectoryActionBuilder intakeSpike3 = shootPos2.fresh()
                .strafeToLinearHeading(new Vector2d(35, -29.3), Math.toRadians(270))
                .strafeTo(new Vector2d(35, -51.1))
                .strafeTo(new Vector2d(35, -27.7));
        TrajectoryActionBuilder shootPos3 = intakeSpike3.fresh()
                .setTangent(-225)
                .splineToLinearHeading(new Pose2d(-21.9, -21.9, Math.toRadians(225)), Math.toRadians(225));


        intake1 = intakeSpike1.build();
        shoot1 = shootPos1.build();
        intake2 = intakeSpike2.build();
        shoot2 = shootPos2.build();
        intake3 = intakeSpike3.build();
        shoot3 = shootPos3.build();


    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (state) {
            case INTAKE1:
                currentAction = intake1.run(packet);
                intake.intake.setPower(-1);
                time.reset();
                if (time.seconds() >= 5){
                    intake.intake.setPower(0);
                }
                if (!currentAction) {
                    state = AutoStates.SHOOT1;
                    time.reset();
                }
                break;
            case SHOOT1:
                currentAction = shoot1.run(packet);
                outtake.outtake.setPower(1);
                if (time.seconds() >= 2){
                    kicker.kicker.setPosition(0);
                    time.reset();
                }
                if (time.seconds() >= 1.5) {
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
                    time.reset();
                    if (time.seconds() >= 5){
                        intake.intake.setPower(0);
                    }
                if (!currentAction){
                    state = AutoStates.SHOOT3;
                    time.reset();
                }
                break;
            case SHOOT3:
                currentAction = shoot3.run(packet);
                outtake.outtake.setPower(1);
                if (time.seconds() >= 2){
                    kicker.kicker.setPosition(0);
                    time.reset();
                }
                if (time.seconds() >= 1.5){
                    kicker.kicker.setPosition(0.3);
                }

                if (!currentAction){
                    state = AutoStates.END;
                }
                break;
            case END:
                // do nothing
                break;

        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}