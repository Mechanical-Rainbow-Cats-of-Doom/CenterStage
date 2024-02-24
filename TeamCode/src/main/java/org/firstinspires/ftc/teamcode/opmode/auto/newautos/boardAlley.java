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

@Autonomous
public class boardAlley extends LinearOpMode {
    public static final Pose2d startPose = new Pose2d(12, -61.75, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        NewLift lift = new NewLift(hardwareMap);
        NewIntake intake = new NewIntake(hardwareMap);

        // Purple pixel movements
        Action left = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(12, -45), Math.toRadians(110))
                .strafeToLinearHeading(new Vector2d(6, -40), Math.toRadians(130))
                .strafeToLinearHeading(new Vector2d(50, -35), Math.toRadians(180))
                .build();
        Action center = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, -32))
                .strafeTo(new Vector2d(20, -35))
                .strafeToLinearHeading(new Vector2d(50, -35), Math.toRadians(180))
                .build();
        Action right = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(70))
                .strafeToLinearHeading(new Vector2d(30, -45), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -35), Math.toRadians(180))
                .build();

        Action cycle = drive.actionBuilder(new Pose2d(50, -35, Math.toRadians(180)))
                .strafeTo(new Vector2d(35, -12))
                .strafeToLinearHeading(new Vector2d(-57, -12), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -12), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -35), Math.toRadians(180))
                .build();




        int visionOutput = -1;

        waitForStart();
        if (isStopRequested()) return;

        Action purplePixel;
        switch (visionOutput) {
            case 0:
                purplePixel = new SequentialAction(
                        left,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LOW),
                        lift.getClawAction(true),
                        new SleepAction(1),
                        lift.getClawAction(false)
                );
                break;
            case 1:
                purplePixel = new SequentialAction(
                        center,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LOW),
                        lift.getClawAction(true),
                        new SleepAction(1),
                        lift.getClawAction(false)
                );
                break;
            case 2:
                purplePixel = new SequentialAction(
                        right,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LOW),
                        lift.getClawAction(true),
                        new SleepAction(1),
                        lift.getClawAction(false)
                );
                break;
            default:
                purplePixel = new SequentialAction(
                        center,
                        lift.moveLiftToPosition(NewLift.LiftPosition.Default.A_LOW),
                        lift.getClawAction(true),
                        new SleepAction(1),
                        lift.getClawAction(false)
                );
                break;
        }


        Actions.runBlocking(
                new SequentialAction(
                        purplePixel
//                        , cycle
                )
        );

    }
}
