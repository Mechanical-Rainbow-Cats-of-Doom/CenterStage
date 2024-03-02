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
public class BlueAudienceAlley extends LinearOpMode {
    public Pose2d startPose;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = false;
        startPose = new Pose2d(-30, 63.75, Math.toRadians(90));

        // initialize camera

        NewLift lift = new NewLift(hardwareMap);
        NewIntake intake = new NewIntake(hardwareMap);

        drive = new MecanumDrive(hardwareMap, startPose);
        int yMultiplier = isRed ? 1 : -1;
        Action centerStart = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(-32, 63.75), Math.toRadians(90))
                .build();
        // Purple pixel movements
        Action rightPurple = drive.actionBuilder(new Pose2d(-32, 63.75, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-39.2, 44), Math.toRadians(40))
                .strafeToLinearHeading(new Vector2d(-31, 50), Math.toRadians(90))
                .build();
        Action rightIntake = drive.actionBuilder(new Pose2d(-31, 50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-31, 13), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-51, 13), Math.toRadians(180))
                .build();

        Action centerPurple = drive.actionBuilder(new Pose2d(-32, 63.75, Math.toRadians(90)))
                .strafeTo(new Vector2d(-35, 37.2))
                .build();
        Action centerIntake = drive.actionBuilder(new Pose2d(-35, 37.2, Math.toRadians(90)))
                .strafeTo(new Vector2d(-50, 40))
                .strafeToLinearHeading(new Vector2d(-51, 13), Math.toRadians(180))
                .build();

        Action leftPurple = drive.actionBuilder(new Pose2d(-32, 63.75, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-41, 38), Math.toRadians(140))
                .strafeToLinearHeading(new Vector2d(-30, 38), Math.toRadians(180))
                .build();
        Action leftIntake = drive.actionBuilder(new Pose2d(-30, 38, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-51, 13), Math.toRadians(180))
                .build();
        Action leftDump = drive.actionBuilder(new Pose2d(54.2, 19, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(56, 25), Math.toRadians(180))
                .build();

        Action cycleLoad = drive.actionBuilder(new Pose2d(-51, 13, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-48, 13), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-51, 13), Math.toRadians(180))
                .build();
        Action cycleBack = drive.actionBuilder(new Pose2d(-51, 13, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(45, 15), Math.toRadians(180))
                .build();
        Action cycleDump = drive.actionBuilder(new Pose2d(45, 15, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(59.2, 18.5), Math.toRadians(190))
                .build();
        Action cycleCenterDump = drive.actionBuilder(new Pose2d(45, 15, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(58.5, 19.5), Math.toRadians(180))
                .build();

        int visionOutput = -1;
        telemetry.addData("Vision detection", visionOutput);

        {
            PropDetector detector = new PropDetector(hardwareMap, "webcam", false,
                    isRed, PropPipeline.PropPipelineRectsProvider.Default.DEFAULT);
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
            detector.close();
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
            case 2:
                purplePixel = new SequentialAction(
                        rightPurple,
                        new ParallelAction(
                                rightIntake,
                                intake.spinIntakeAction(5, () -> 0.75, NewIntake.DefaultHeight.PIXEL_5)
                        ),
                        new ParallelAction(
                                intake.spinIntakeAction(2, () -> 1, NewIntake.DefaultHeight.UP),
                                cycleLoad
                        ),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        intake.spinIntakeAction(1.5, () -> -1, NewIntake.DefaultHeight.UP)
                                ),
                                cycleBack
                        ),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_RIGHT_VLOW_RIGHTDUMP),
                        cycleDump,
                        lift.getClawAction(true),
                        new SleepAction(1.5),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_RIGHT_VLOW_RIGHTDUMP_UPPER),
                        lift.getClawAction(false),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
            case 1:
                purplePixel = new SequentialAction(
                        centerPurple,
                        new ParallelAction(
                                centerIntake,
                                intake.spinIntakeAction(4, () -> 0.75, NewIntake.DefaultHeight.PIXEL_5)
                        ),
                        new ParallelAction(
                                intake.spinIntakeAction(2, () -> 1, NewIntake.DefaultHeight.UP),
                                cycleLoad
                        ),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.spinIntakeAction(1.5, () -> -1, NewIntake.DefaultHeight.UP)
                                ),
                                cycleBack
                        ),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_RIGHT_VLOW_MIDDLEDUMP),
                        cycleCenterDump,
                        lift.getClawAction(true),
                        new SleepAction(1.5),
                        lift.getClawAction(false),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
            case 0:
                purplePixel = new SequentialAction(
                        leftPurple,
                        new ParallelAction(
                                leftIntake,
                                intake.spinIntakeAction(3, () -> 0.6, NewIntake.DefaultHeight.PIXEL_5)
                        ),
                        new ParallelAction(
                                intake.spinIntakeAction(2, () -> 1, NewIntake.DefaultHeight.UP),
                                cycleLoad
                        ),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        intake.spinIntakeAction(1.5, () -> -1, NewIntake.DefaultHeight.UP)
                                ),
                                cycleBack
                        ),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_RIGHT_VLOW_LEFTDUMP),
                        cycleDump,
                        leftDump, // if moving later, you need to account for the fact that right goes to a different position
                        lift.getClawAction(true),
                        new SleepAction(1.5),
                        lift.getClawAction(false),
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
            default:
                purplePixel = new SequentialAction(
//                        centerPurple,
//                        centerIntake,
//                        intake.setIntakeHeightAction(NewIntake.DefaultHeight.PIXEL_HOVER),
//                        intake.spinIntakeAction(0.3, () -> 0.5, NewIntake.DefaultHeight.PIXEL_HOVER),
//                        intake.spinIntakeAction(0.2, () -> 0.5, NewIntake.DefaultHeight.PIXEL_5),
//                        new SleepAction(0.2),
//                        intake.spinIntakeAction(0.8, () -> 0.5, NewIntake.DefaultHeight.PIXEL_5),
//                        new ParallelAction(
//                                intake.spinIntakeAction(3, () -> 1, NewIntake.DefaultHeight.UP),
//                                cycleBack
//                        ),
//                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LEFT_VLOW_MIDDLEDUMP),
//                        cycleDump,
//                        lift.getClawAction(true),
//                        new SleepAction(1.5),
//                        lift.getClawAction(false),
//                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN)
                );
                break;
        }

        Actions.runBlocking(
                new ParallelAction(
                        lift.runPeriodicAction(),
                        new SequentialAction(
                                centerStart,
                                purplePixel
                        )
                )

        );

    }
}
