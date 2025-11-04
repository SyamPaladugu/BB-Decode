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
//       RED SIDE SCORE 3
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49.9, -49.7, Math.toRadians(55)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-12.2,-27,Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.1)
                .lineToY(-51.1)
                .waitSeconds(0.1)
                .lineToY(-27)
                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(-21.9,-21.9,Math.toRadians(225)),Math.toRadians(0))
                .waitSeconds(0.1)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}