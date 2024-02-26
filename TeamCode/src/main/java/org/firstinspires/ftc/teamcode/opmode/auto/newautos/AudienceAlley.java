package org.firstinspires.ftc.teamcode.opmode.auto.newautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tool.NewIntake;
import org.firstinspires.ftc.teamcode.tool.NewLift;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@Autonomous
public class AudienceAlley extends LinearOpMode {
    public Pose2d startPose;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = SwapSides.isRed;
        startPose = new Pose2d(-35, -63.75*(isRed ? 1 : -1), Math.toRadians(270));

        // initialize camera
        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                isRed, PropPipeline.PropPipelineRectsProvider.Default.DEFAULT);

        NewLift lift = new NewLift(hardwareMap);
        NewIntake intake = new NewIntake(hardwareMap);

        drive = new MecanumDrive(hardwareMap, startPose);
        int yMultiplier = isRed ? 1 : -1;

        // Purple pixel movements
        Action leftPurple = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-41, -37*(isRed ? 1 : -1)), Math.toRadians(290))
                .strafeToLinearHeading(new Vector2d(-35, -38*(isRed ? 1 : -1)), Math.toRadians(180))
                .build();
        Action leftIntake = drive.actionBuilder(new Pose2d(-35, -38*(isRed ? 1 : -1), Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-35, -13*(isRed ? 1 : -1)), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-53, -13*(isRed ? 1 : -1)), Math.toRadians(180))
                .build();

        Action centerPurple = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-35, -32*(isRed ? 1 : -1)))
                .build();
        Action centerIntake = drive.actionBuilder(new Pose2d(-35, -32*(isRed ? 1 : -1), Math.toRadians(270)))
                .strafeTo(new Vector2d(-45, -40*(isRed ? 1 : -1)))
                .strafeToLinearHeading(new Vector2d(-53, -13*(isRed ? 1 : -1)), Math.toRadians(180))
                .build();

        Action rightPurple = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-35, -45*(isRed ? 1 : -1)), Math.toRadians(250))
                .build();
        Action rightIntake = drive.actionBuilder(new Pose2d(-35, -45*(isRed ? 1 : -1), Math.toRadians(250)))
                .strafeToLinearHeading(new Vector2d(-29, -40*(isRed ? 1 : -1)), Math.toRadians(230))
                .strafeToLinearHeading(new Vector2d(-53, -13*(isRed ? 1 : -1)), Math.toRadians(180))
                .build();
        Action rightDump = drive.actionBuilder(new Pose2d(53.5, -19*(isRed ? 1 : -1), Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(53.5, -24*(isRed ? 1 : -1)), Math.toRadians(250))
                .build();

        Action cycleBack = drive.actionBuilder(new Pose2d(-53, -13*(isRed ? 1 : -1), Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(45, -15*(isRed ? 1 : -1)), Math.toRadians(180))
                .build();
        Action cycleDump = drive.actionBuilder(new Pose2d(45, -15*(isRed ? 1 : -1), Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(54.2, -19*(isRed ? 1 : -1)), Math.toRadians(180))
                .build();

        int visionOutput = -1;
        telemetry.addData("Vision detection", visionOutput);

        while (!isStopRequested() && !opModeIsActive()) {
            float startTime = System.currentTimeMillis() / 1000f;
            visionOutput = detector.run(() -> {
                int time = (int) ((System.currentTimeMillis() - startTime) / 10f) % 4;
//                telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
//                        (time > 2 ? "." : "") +
//                        (time > 3 ? "." : ""));
//                telemetry.update();
            });
            telemetry.addData("Vision detection", visionOutput);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // TODO:
//        float startTime = System.currentTimeMillis() / 1000f;
//        int detection = detector.run(() -> {
//            int time = (int) ((System.currentTimeMillis() - startTime) / 10f) % 4;
//            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
//                    (time > 2 ? "." : "") +
//                    (time > 3 ? "." : ""));
//            telemetry.update();
//        });

        Action purplePixel;
        switch (visionOutput) {
            case 0:
                purplePixel = new SequentialAction(
                        leftPurple,
                        new ParallelAction(
                                leftIntake,
                                intake.setIntakeHeightAction(NewIntake.DefaultHeight.PIXEL_HOVER)
                        ),
                        intake.spinIntakeAction(1, () -> 0.75, NewIntake.DefaultHeight.PIXEL_5),
                        intake.setIntakeHeightAction(NewIntake.DefaultHeight.UP),
                        cycleBack,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LEFT_VLOW_LEFTDUMP),
                        cycleDump,
                        lift.getClawAction(true),
                        new SleepAction(1.5),
                        lift.getClawAction(false),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
            case 1:
                purplePixel = new SequentialAction(
                        centerPurple,
                        new ParallelAction(
                                centerIntake,
                                intake.setIntakeHeightAction(NewIntake.DefaultHeight.PIXEL_HOVER)
                        ),
                        intake.spinIntakeAction(1, () -> 0.75, NewIntake.DefaultHeight.PIXEL_5),
                        intake.setIntakeHeightAction(NewIntake.DefaultHeight.UP),
                        cycleBack,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LEFT_VLOW_MIDDLEDUMP),
                        cycleDump,
                        lift.getClawAction(true),
                        new SleepAction(1.5),
                        lift.getClawAction(false),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
            case 2:
                purplePixel = new SequentialAction(
                        rightPurple,
                        new ParallelAction(
                                rightIntake,
                                intake.setIntakeHeightAction(NewIntake.DefaultHeight.PIXEL_HOVER)
                        ),
                        intake.spinIntakeAction(1, () -> 0.75, NewIntake.DefaultHeight.PIXEL_5),
                        intake.setIntakeHeightAction(NewIntake.DefaultHeight.UP),
                        cycleBack,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LEFT_VLOW_RIGHTDUMP),
                        cycleDump,
                        rightDump, // if moving later, you need to account for the fact that right goes to a different position
                        lift.getClawAction(true),
                        new SleepAction(1.5),
                        lift.getClawAction(false),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
            default:
                purplePixel = new SequentialAction(
                        centerPurple,
                        new ParallelAction(
                                centerIntake,
                                intake.setIntakeHeightAction(NewIntake.DefaultHeight.PIXEL_HOVER)
                        ),
                        intake.spinIntakeAction(1, () -> 0.75, NewIntake.DefaultHeight.PIXEL_5),
                        intake.setIntakeHeightAction(NewIntake.DefaultHeight.UP),
                        cycleBack,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LEFT_VLOW_MIDDLEDUMP),
                        cycleDump,
                        lift.getClawAction(true),
                        new SleepAction(1.5),
                        lift.getClawAction(false),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
        }

        Actions.runBlocking(
                new SequentialAction(
                        purplePixel
                )
        );

    }
}
