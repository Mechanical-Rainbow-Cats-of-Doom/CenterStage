package org.firstinspires.ftc.teamcode.opmode.auto.newautos;

import com.acmerobotics.roadrunner.Action;
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

@Autonomous
public class boardAlley extends LinearOpMode {
    public static final Pose2d startPose = new Pose2d(12, -61.75, Math.toRadians(270));
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        NewLift lift = new NewLift(hardwareMap);
        NewIntake intake = new NewIntake(hardwareMap);
        PixelSensor pixelSensor = new PixelSensor(hardwareMap);

        // Purple pixel movements
        Action left = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(12, -45), Math.toRadians(290))
                .strafeToLinearHeading(new Vector2d(6, -40), Math.toRadians(310))
                .strafeToLinearHeading(new Vector2d(55, -35), Math.toRadians(180))
                .build();
        Action center = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, -32))
                .strafeTo(new Vector2d(20, -35))
                .strafeToLinearHeading(new Vector2d(55, -35), Math.toRadians(180))
                .build();
        Action right = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(250))
                .strafeToLinearHeading(new Vector2d(30, -45), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(55, -35), Math.toRadians(180))
                .build();

        Action cycleDown = drive.actionBuilder(new Pose2d(50, -35, Math.toRadians(180)))
                .strafeTo(new Vector2d(35, -15))
                .strafeToLinearHeading(new Vector2d(-50, -15), Math.toRadians(180))
                .build();

        Action cycleBack = drive.actionBuilder(new Pose2d(-50, -15, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(50, -15), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(55, -35), Math.toRadians(180))
                .build();




        int visionOutput = -1;

        waitForStart();
        lift.periodic();
        if (isStopRequested()) return;

        Action purplePixel;
        switch (visionOutput) {
            case 0:
                purplePixel = new SequentialAction(
                        left,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_MIDDLE_VLOW_LEFTDUMP),
                        lift.getClawAction(true),
                        new SleepAction(1),
                        lift.getClawAction(false)
                );
                break;
            case 1:
                purplePixel = new SequentialAction(
                        center,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_VLOW_MIDDLEDUMP_RIGHT_LEAN),
                        lift.getClawAction(true),
                        new SleepAction(1),
                        lift.getClawAction(false)
                );
                break;
            case 2:
                purplePixel = new SequentialAction(
                        right,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_MIDDLE_VLOW_RIGHTDUMP),
                        lift.getClawAction(true),
                        new SleepAction(1),
                        lift.getClawAction(false)
                );
                break;
            default:
                purplePixel = new SequentialAction(
                        center,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_VLOW_MIDDLEDUMP_RIGHT_LEAN),
                        lift.getClawAction(true),
                        new SleepAction(1),
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
                new SequentialAction(
                        purplePixel,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.DOWN),
                        cycleDown,
                        intake.spinIntakeAction(5, NewIntake.DefaultHeight.PIXEL_4),
                        cycleBack
                )
        );

    }
}
