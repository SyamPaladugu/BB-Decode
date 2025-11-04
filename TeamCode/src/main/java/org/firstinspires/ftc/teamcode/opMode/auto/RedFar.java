package org.firstinspires.ftc.teamcode.opMode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opMode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.opMode.teleOp.MecanumTeleOp;


@Autonomous(name="RedFar",group = "Autonomous")
public class RedFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPos = new Pose2d(new Vector2d(53.6,5.5),Math.toRadians(180));
        //Insert drive Code here
        MecanumDrive drive = new MecanumDrive(hardwareMap,startPos);
        waitForStart();
        Action strafe = drive.actionBuilder(startPos)
                .strafeTo(new Vector2d(53.6,32.8))
                .build();
    }
}
