
        package com.example.meepmeeptesting;

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

        // myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0,Math.toRadians(126)))

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(57, 19,Math.toRadians(160)))
                //----------Grab the first balls-----------

                        .splineToLinearHeading(new Pose2d(34, 25, Math.toRadians(90)), Math.toRadians(180))
                        .setReversed(true)
                        .lineToY(60)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(50, 13, Math.toRadians(159)), Math.toRadians(220))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(15, 25, Math.toRadians(90)), Math.toRadians(180))
                        .setReversed(true)
                        .lineToY(60)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(50, 13, Math.toRadians(159)), Math.toRadians(220))

                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-13, 25, Math.toRadians(90)), Math.toRadians(180))
                        .setReversed(true)
                        .lineToY(60)

                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(50, 13, Math.toRadians(159)), Math.toRadians(220))
                .build());


                meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}