package com.mrcod.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(700);

        final double width = 18, height = 18;
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(width, height)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(-36,(height/2)-72, Math.toRadians(90)))
//                .strafeTo(new Vector2d())
                        .strafeTo(new Vector2d(-36, -12))
                        .strafeToLinearHeading(new Vector2d(-55, -12), Math.toRadians(180))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(24, -12))
                        .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))
                        .waitSeconds(2)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }


}