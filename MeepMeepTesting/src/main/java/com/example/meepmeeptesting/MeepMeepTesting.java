package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.HeadingPath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49.9, -49.7, Math.toRadians(55)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-23.3,-27,Math.toRadians(270)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-15,-34.5,Math.toRadians(290)),Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(-14.5,-49.3,Math.toRadians(270)), Math.toRadians(200))
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(-21.9,-22,Math.toRadians(225)),Math.toRadians(140))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(5.3,-27.8,Math.toRadians(290)), Math.toRadians(310))
                .splineToSplineHeading(new Pose2d(11.5,-52.5,Math.toRadians(270)),Math.toRadians(270))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-22.9,-22.7), Math.toRadians(225))
                .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(24.1,-28.6,Math.toRadians(290)), Math.toRadians(330))
                .splineToLinearHeading(new Pose2d(34.9, -52.9,Math.toRadians(270)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-23.8,-23.8), Math.toRadians(225))
                //.waitSeconds(1)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}