package com.mrcod.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(700);

        final double width = 17.5, height = 16.5;
        DefaultBotBuilder botBuilder = new DefaultBotBuilder(meepMeep).setDimensions(width, height)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826);

        RoadRunnerBotEntity boardLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.75, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(12, -45, Math.toRadians(110)))
                                .lineToLinearHeading(new Pose2d(6, -40, Math.toRadians(130)))
                                .build());

        RoadRunnerBotEntity boardCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.75, Math.toRadians(90)))
                                .lineTo(new Vector2d(12, -32))
                                .lineTo(new Vector2d(20, -35))
                                .lineToLinearHeading(new Pose2d(50, -35, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -59, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(60, -59, Math.toRadians(180)))
                                .build());

        RoadRunnerBotEntity boardRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.75, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(18, -40, Math.toRadians(70)))
                                .lineToLinearHeading(new Pose2d(50, -59, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -59, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -35, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -59, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -59, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -59, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -59, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(60, -59, Math.toRadians(180)))
                                .build());

        RoadRunnerBotEntity audienceRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -63.75, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-35, -45, Math.toRadians(70)))
                                .lineToLinearHeading(new Pose2d(-29, -40, Math.toRadians(50)))
                                .build());

        RoadRunnerBotEntity audienceCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -63.75, Math.toRadians(90)))
                                .lineTo(new Vector2d(-35, -32))
                                .lineTo(new Vector2d(-35, -40))
                                .lineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)))
                                .build());

        RoadRunnerBotEntity audienceLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.23195266731826)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -63.75, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-41, -37, Math.toRadians(110)))
                                .lineToLinearHeading(new Pose2d(-35, -38, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(boardCenter)
                .addEntity(boardLeft)
                .addEntity(boardRight)
                .addEntity(audienceRight)
                .addEntity(audienceCenter)
                .addEntity(audienceLeft)
                .start();
    }


}