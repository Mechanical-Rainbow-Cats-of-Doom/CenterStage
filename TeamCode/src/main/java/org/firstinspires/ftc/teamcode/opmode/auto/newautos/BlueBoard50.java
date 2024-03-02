package org.firstinspires.ftc.teamcode.opmode.auto.newautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
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

@Autonomous(group = "50 Points")
public class BlueBoard50 extends LinearOpMode {
    public Pose2d startPose;
    public PoseMap poseMap = pose ->
            new Pose2dDual<>(pose.position.x,
                    pose.position.y.unaryMinus(),
                    pose.heading.inverse());

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = false;
        // initialize camera
        startPose = new Pose2d(14, 61.75, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        NewLift lift = new NewLift(hardwareMap);
        NewIntake intake = new NewIntake(hardwareMap);
        PixelSensor pixelSensor = new PixelSensor(hardwareMap);

        Action park = drive.actionBuilder(new Pose2d(56, 35, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(45, 36), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(54, 61), Math.toRadians(180))
                .build();

        // Purple pixel movements
        Action leftPurple = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(15, 40), Math.toRadians(130))
                .strafeToLinearHeading(new Vector2d(15, 48), Math.toRadians(90))
                .build();
        Action leftDump = drive.actionBuilder(new Pose2d(15, 48, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(28, 48), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(56, 34), Math.toRadians(180))
                .build();

        Action centerPurple = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(12, 35.6))
                .strafeTo(new Vector2d(20, 40))
                .build();
        Action centerDump = drive.actionBuilder(new Pose2d(20, 40, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(56.5, 37), Math.toRadians(180))
                .build();

        Action rightPurple = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(18, 43), Math.toRadians(50))
                .strafeToLinearHeading(new Vector2d(11, 40), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(10, 40), Math.toRadians(0))
                .build();
        Action rightDump = drive.actionBuilder(new Pose2d(10, 40, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(40, 40), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(55, 35), Math.toRadians(180))
                .build();

        Action middleToWall = drive.actionBuilder(new Pose2d(55.5, -37, Math.toRadians(180)), poseMap)
                .strafeToLinearHeading(new Vector2d(12, -61), Math.toRadians(170))
                .build();

        Action cycleDown = drive.actionBuilder(new Pose2d(12, -61, Math.toRadians(170)), poseMap)
                .strafeToLinearHeading(new Vector2d(-52.5, -60), Math.toRadians(180))
                .strafeTo(new Vector2d(-54, -35))
                .build();

        Action cycleBack = drive.actionBuilder(new Pose2d(-54, -35, Math.toRadians(180)), poseMap)
                .strafeToLinearHeading(new Vector2d(-35, -61), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(40, -60), Math.toRadians(180))
                .build();

        Action cycleDump = drive.actionBuilder(new Pose2d(40, -62, Math.toRadians(180)), poseMap)
                .strafeToLinearHeading(new Vector2d(55, -50), Math.toRadians(180))
                .build();


        int visionOutput = -1;

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
        lift.periodic();
        if (isStopRequested()) return;

        Action purplePixel;
        switch (visionOutput) {
            case 0:
                purplePixel = new SequentialAction(
                        leftPurple,
                        new ParallelAction(
                                leftDump,
                                lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_MIDDLE_VLOW_LEFTDUMP)
                        ),
                        lift.getClawAction(true),
                        new SleepAction(0.5),
                        lift.getClawAction(false)
                );
                break;
            case 1:
                purplePixel = new SequentialAction(
                        centerPurple,
                        new ParallelAction(
                                centerDump,
                                lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_VLOW_MIDDLEDUMP_RIGHT_LEAN)
                        ),
                        lift.getClawAction(true),
                        new SleepAction(0.5),
                        lift.getClawAction(false)
                );
                break;
            case 2:
                purplePixel = new SequentialAction(
                        rightPurple,
                        new ParallelAction(
                                rightDump,
                                lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_MIDDLE_VLOW_RIGHTDUMP)
                        ),
                        lift.getClawAction(true),
                        new SleepAction(0.5),
                        lift.getClawAction(false)
                );
                break;
            default:
                purplePixel = new SequentialAction(
                        centerPurple,
                        centerDump,
                        lift.getClawAction(true),
                        new SleepAction(0.5),
                        lift.getClawAction(false)
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
                new ParallelAction(
                        lift.runPeriodicAction(),
                        new SequentialAction(
                                purplePixel,
                                new SleepAction(1),
                                new ParallelAction(
                                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN),
                                        park
                                )
                        )
                )

        );

    }
}
