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
import org.firstinspires.ftc.teamcode.tool.PixelSensor;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@Autonomous
public class BoardAlley extends LinearOpMode {
    public Pose2d startPose;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = SwapSides.isRed;
        // initialize camera
        startPose = new Pose2d(12, -61.75 * (isRed ? 1 : -1), Math.toRadians(270) - (isRed ? 0 : Math.toRadians(180)));
        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                isRed, PropPipeline.PropPipelineRectsProvider.Default.DEFAULT);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        NewLift lift = new NewLift(hardwareMap);
        NewIntake intake = new NewIntake(hardwareMap);
        PixelSensor pixelSensor = new PixelSensor(hardwareMap);

        // Purple pixel movements
        Action leftPurple = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(18, -48 * (isRed ? 1 : -1)), Math.toRadians(280))
                .strafeToLinearHeading(new Vector2d(3, -43 * (isRed ? 1 : -1)), Math.toRadians(360))
                .build();
        Action leftDump = drive.actionBuilder(new Pose2d(6, -40 * (isRed ? 1 : -1), Math.toRadians(310)))
                .strafeToLinearHeading(new Vector2d(55, -37 * (isRed ? 1 : -1)), Math.toRadians(170))
                .build();

        Action centerPurple = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, -36.4 * (isRed ? 1 : -1)))
                .strafeTo(new Vector2d(20, -40 * (isRed ? 1 : -1)))
                .build();
        Action centerDump = drive.actionBuilder(new Pose2d(20, -35 * (isRed ? 1 : -1), Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(55, -37 * (isRed ? 1 : -1)), Math.toRadians(180))
                .build();

        Action rightPurple = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(16.5, -40 * (isRed ? 1 : -1)), Math.toRadians(230))
                .strafeToLinearHeading(new Vector2d(16.5, -48 * (isRed ? 1 : -1)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(30, -45 * (isRed ? 1 : -1)), Math.toRadians(180))
                .build();
        Action rightDump = drive.actionBuilder(new Pose2d(30, -45, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(55, -37 * (isRed ? 1 : -1)), Math.toRadians(180))
                .build();

        Action cycleDown = drive.actionBuilder(new Pose2d(55, -37 * (isRed ? 1 : -1), Math.toRadians(180)))
                .strafeTo(new Vector2d(35, -12 * (isRed ? 1 : -1)))
                .strafeToLinearHeading(new Vector2d(-52.5, -13.5 * (isRed ? 1 : -1)), Math.toRadians(180))
                .build();

        Action cycleBack = drive.actionBuilder(new Pose2d(-52.5, -13.5 * (isRed ? 1 : -1), Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(45, -15 * (isRed ? 1 : -1)), Math.toRadians(180))
                .build();

        Action cycleDump = drive.actionBuilder(new Pose2d(45, -15 * (isRed ? 1 : -1), Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(54.5, -19 * (isRed ? 1 : -1)), Math.toRadians(180))
                .build();


        int visionOutput = -1;

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
        lift.periodic();
        if (isStopRequested()) return;

        Action purplePixel;
        switch (visionOutput) {
            case 0:
                purplePixel = new SequentialAction(
                        leftPurple,
                        new ParallelAction(
                                leftDump
//                                lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_MIDDLE_VLOW_LEFTDUMP)
                        ),
//                        lift.getClawAction(true),
                        new SleepAction(0.5)
//                        lift.getClawAction(false)
                );
                break;
            case 1:
                purplePixel = new SequentialAction(
                        centerPurple,
                        new ParallelAction(
                                centerDump
//                                lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_VLOW_MIDDLEDUMP_RIGHT_LEAN)
                        ),
//                        lift.getClawAction(true),
                        new SleepAction(0.5)
//                        lift.getClawAction(false)
                );
                break;
            case 2:
                purplePixel = new SequentialAction(
                        rightPurple,
                        new ParallelAction(
                                rightDump
//                                lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_MIDDLE_VLOW_RIGHTDUMP)
                        ),
//                        lift.getClawAction(true),
                        new SleepAction(0.5)
//                        lift.getClawAction(false)
                );
                break;
            default:
                purplePixel = new SequentialAction(
                        centerPurple,
                        new ParallelAction(
                                centerDump
//                                lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_VLOW_MIDDLEDUMP_RIGHT_LEAN)
                        ),
//                        lift.getClawAction(true),
                        new SleepAction(0.5)
//                        lift.getClawAction(false)
                );
                break;
        }

//        switch (visionOutput) {
//            case 0:
//                purplePixel = left;
//                break;
//            case 1:
//                purplePixel = center;
//                break;
//            case 2:
//                purplePixel = right;
//                break;
//            default:
//                purplePixel = center;
//                break;
//        }

        Actions.runBlocking(
                new SequentialAction(
                        purplePixel
//                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                ));
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                cycleDown,
                                intake.setIntakeHeightAction(NewIntake.DefaultHeight.PIXEL_HOVER)
                        ),
                        intake.spinIntakeAction(1, () -> 0.5, NewIntake.DefaultHeight.PIXEL_5),
                        intake.spinIntakeAction(1.5, () -> 0.8, NewIntake.DefaultHeight.PIXEL_4),
                        intake.setIntakeHeightAction(NewIntake.DefaultHeight.UP),
                        new ParallelAction(
                                intake.spinIntakeAction(2, NewIntake.State.BACKWARD, NewIntake.DefaultHeight.UP),
                                cycleBack
                        ),
//                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LEFT_LOW),
                        cycleDump,
                        new SleepAction(0.4),
//                        lift.getClawAction(true),
                        new SleepAction(1.5)
//                        lift.getClawAction(false),
//                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                )
        );

    }
}
