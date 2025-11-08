package org.firstinspires.ftc.teamcode.opMode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "BlueFar", group = "Autonomous")
public class BlueSideFar extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-49.9, -49.7, Math.toRadians(55));

    MecanumDrive follower;

    Action intake1, shoot1, intake2, shoot2, intake3, shoot3;

    public ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new MecanumDrive(hardwareMap, initialPose);

        build_paths();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        intake1,
                        shoot1,
                        intake2,
                        shoot2,
                        intake3,
                        shoot3
                )
        );


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
        TrajectoryActionBuilder shootPos1 = intakeSpike1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-21.9, -21.9, Math.toRadians(225)), Math.toRadians(0))
                .waitSeconds(2);
        TrajectoryActionBuilder intakeSpike2 = shootPos1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10.9, -27.7), Math.toRadians(270))
                .waitSeconds(0.7)
                .strafeTo(new Vector2d(11.5, -51.1))
                .strafeTo(new Vector2d(10.9, -27.7))
                .waitSeconds(0.1);
        TrajectoryActionBuilder shootPos2 = intakeSpike2.endTrajectory().fresh()
                .setTangent(-225)
                .splineToLinearHeading(new Pose2d(-21.9, -21.9, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(2);
        TrajectoryActionBuilder intakeSpike3 = shootPos2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -29.3), Math.toRadians(270))
                .strafeTo(new Vector2d(35, -51.1))
                .strafeTo(new Vector2d(35, -27.7));
        TrajectoryActionBuilder shootPos3 = intakeSpike3.endTrajectory().fresh()
                .setTangent(-225)
                .splineToLinearHeading(new Pose2d(-21.9, -21.9, Math.toRadians(225)), Math.toRadians(225));


        intake1 = intakeSpike1.build();
        shoot1 = shootPos1.build();
        intake2 = intakeSpike2.build();
        shoot2 = shootPos2.build();
        intake3 = intakeSpike3.build();
        shoot3 = shootPos3.endTrajectory().build();


    }
}