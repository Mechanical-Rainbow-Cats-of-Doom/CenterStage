package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.BuildersKt;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.util.DelayStorage;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tool.old.OldLift;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@Autonomous
@Disabled
public class BlueBoardPlacePixelAuto extends LinearOpMode {
    protected boolean isRed = false;
    protected boolean closeRightTurn = false;

    @Override
    public void runOpMode() throws InterruptedException {
//        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
//                isRed, PropPipeline.PropPipelineRectsProvider.Default.RED_BOARD_SIDE);
        OldLift lift = new OldLift(hardwareMap, null, false, false, null);
        Pose2d startPose = new Pose2d(12, -61.75, Math.toRadians(90));
        final Pose2d secondInitialPose = new Pose2d(12, -50, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action left = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(12,-45), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6, -40), Math.toRadians(130))
                .strafeToLinearHeading(new Vector2d(12,-45), Math.toRadians(90))
                .strafeToLinearHeading(secondInitialPose.position, secondInitialPose.heading)
                .build();
        Action middle = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(12, -32))
                .strafeToLinearHeading(secondInitialPose.position, secondInitialPose.heading)
                .build();
        Action right = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(12,-42), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(closeRightTurn ? 45 : 33))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(12,-41), Math.toRadians(90))
                .strafeToLinearHeading(secondInitialPose.position, secondInitialPose.heading)
                .build();
        double boardX = 48;
        PoseMap originalDirection = pose -> new Pose2dDual<>(pose.position.x, pose.position.y, pose.heading);
        PoseMap oppositeDirection = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
        Pose2d finalLeft = new Pose2d(boardX, -27, Math.toRadians(180));
        Pose2d finalMiddle = new Pose2d(boardX, -34, Math.toRadians(180));
        Pose2d finalRight = new Pose2d(boardX, -41, Math.toRadians(180));
        Action leftBoard = drive.actionBuilder(secondInitialPose, isRed ? originalDirection : oppositeDirection)
                .strafeToLinearHeading(finalLeft.position, finalLeft.heading)
                .build();
        Action middleBoard = drive.actionBuilder(secondInitialPose, isRed ? originalDirection : oppositeDirection)
                .strafeToLinearHeading(finalMiddle.position, finalMiddle.heading)
                .build();
        Action rightBoard = drive.actionBuilder(secondInitialPose, isRed ? originalDirection : oppositeDirection)
                .strafeToLinearHeading(finalRight.position, finalRight.heading)
                .build();

        final Vector2d finalFinalPosition = new Vector2d(boardX, -65);

        Action leftBoardEnd = drive.actionBuilder(finalLeft, isRed ? originalDirection : oppositeDirection)
                .strafeTo(finalFinalPosition)
                .build();
        Action middleBoardEnd = drive.actionBuilder(finalMiddle, isRed ? originalDirection : oppositeDirection)
                .strafeTo(finalFinalPosition)
                .build();
        Action rightBoardEnd = drive.actionBuilder(finalRight, isRed ? originalDirection : oppositeDirection)
                .strafeTo(finalFinalPosition)
                .build();

        Timing.Timer timer = DelayStorage.getTimer();
        waitForStart();
        lift.toggleClawOpen();

        timer.start();
        float startTime = System.currentTimeMillis() / 1000f;
        int detection =0;
//                detector.run(() -> {
//            int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
//            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
//                    (time > 2 ? "." : "") +
//                    (time > 3 ? "." : ""));
//            telemetry.update();
//        });

        while(!timer.done()) {
            telemetry.addLine("Waiting for delay");
            telemetry.addData("Time Left", timer.remainingTime() / 1000);
            telemetry.update();
        }
//        DelayStorage.waitForDelay();
        switch (detection) {
            case 0:
                Actions.runBlocking(left);
                if(!isRed) {
                    drive.pose = BuildersKt.map(oppositeDirection, drive.pose);
                    // this genuinely hurts my head
                    detection = 2;
                    Actions.runBlocking(rightBoard);
                } else {
                    Actions.runBlocking(leftBoard);
                }
                break;
            case 1:
                Actions.runBlocking(middle);
                if(!isRed) {
                    drive.pose = BuildersKt.map(oppositeDirection, drive.pose);
                }
                Actions.runBlocking(middleBoard);
                break;
            case 2:
                Actions.runBlocking(right);
                if(!isRed) {
                    drive.pose = BuildersKt.map(oppositeDirection, drive.pose);
                    detection = 0;
                    Actions.runBlocking(leftBoard);
                } else {
                    Actions.runBlocking(rightBoard);
                }
                break;
        }
        lift.setPosition(OldLift.LiftPosition.Default.DOWN);
        while(!lift.getState().isFinished()) {
            lift.periodic();
        }
        lift.setPosition(OldLift.LiftPosition.Default.LOW);
        while(!lift.getState().isFinished()) {
            lift.periodic();
        }
        lift.setPosition(OldLift.LiftPosition.Default.DOWN);
        while(!lift.getState().isFinished()) {
            lift.periodic();
        }
        switch (detection) {
            case 0:
                Actions.runBlocking(leftBoardEnd);
                break;
            case 1:
                Actions.runBlocking(middleBoardEnd);
                break;
            case 2:
                Actions.runBlocking(rightBoardEnd);
                break;
        }
    }
}
