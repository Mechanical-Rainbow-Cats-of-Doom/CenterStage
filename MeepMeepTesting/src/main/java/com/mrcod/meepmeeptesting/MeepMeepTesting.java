package com.mrcod.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(700);

        final double width = 17.5, height = 16.5;
        DefaultBotBuilder botBuilder = new DefaultBotBuilder(meepMeep)
                .setDimensions(width, height)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826);
        RoadRunnerBotEntity left = botBuilder.build();
        RoadRunnerBotEntity center = botBuilder.build();
        RoadRunnerBotEntity right = botBuilder.build();

        left.runAction(left.getDrive().actionBuilder(new Pose2d(12,-63.75, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(12,-45), Math.toRadians(110))
                .strafeToLinearHeading(new Vector2d(6, -40), Math.toRadians(130))
                .build());
        center.runAction(center.getDrive().actionBuilder(new Pose2d(12,-63.75, Math.toRadians(90)))
                .strafeTo(new Vector2d(12, -32))
                .build());
        right.runAction(right.getDrive().actionBuilder(new Pose2d(12,-63.75, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(60))
                .strafeToLinearHeading(new Vector2d(48, -41.5), Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(center)
                .addEntity(left)
                .addEntity(right)
                .start();
    }


}