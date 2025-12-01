package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(0));

    //Store whether we're starting from auto or a fresh start
    public static boolean poseFromAuto = false;
}