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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 13, Math.toRadians(165)))
                //----------Grab the first balls-----------
                .strafeTo(new Vector2d(37,20))
                .turn(Math.toRadians(-75))
                .lineToY(54)/*
                .strafeTo(new Vector2d(-50,47))
                .turn(Math.toRadians(44.1))
                //----------Grab the second balls---------
                .strafeTo(new Vector2d(13,20))
                .turn(Math.toRadians(-44.1))
                .lineToY(54)
                .strafeTo(new Vector2d(-18,16))
                .turn(Math.toRadians(44.1))
                //----------Grab the third balls
                .strafeTo(new Vector2d(36,28))
                .turn(Math.toRadians(-44.1))
                .lineToY(54)
                .strafeTo(new Vector2d(-18,16))
                .turn(Math.toRadians(44.1))*/
                .build());
        //-----------BlueSide----------
       /* myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, -47, -14.9))
                //----------Grab the first balls-----------
                .strafeTo(new Vector2d(-12,-20))
                .turn(Math.toRadians(44.1))
                .lineToY(-56)
                .strafeTo(new Vector2d(-52,-47))
                .turn(Math.toRadians(-44.1))
                //----------Grab the second balls---------
                .strafeTo(new Vector2d(13,-20))
                .turn(Math.toRadians(44.1))
                .lineToY(-56)
                .strafeTo(new Vector2d(-18,-16))
                .turn(Math.toRadians(-44.1))
                //----------Grab the third balls
                .strafeTo(new Vector2d(36,-28))
                .turn(Math.toRadians(44.1))
                .lineToY(-56)
                .strafeTo(new Vector2d(-18,-16))
                .turn(Math.toRadians(-44.1))
                .build()); */


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}