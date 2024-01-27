package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.BuildersKt;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tool.Lift;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@Autonomous
public class BlueBoardPlacePixelAuto extends LinearOpMode {
    protected boolean isRed = false;
    protected boolean closeRightTurn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                isRed, PropPipeline.PropPipelineRectsProvider.Default.RED_BOARD_SIDE);
        Lift lift = new Lift(hardwareMap, null, false, false, null);
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
                .strafeTo(new Vector2d(12, -33))
                .strafeToLinearHeading(secondInitialPose.position, secondInitialPose.heading)
                .build();
        Action right = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(12,-41), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(closeRightTurn ? 45 : 35))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(12,-41), Math.toRadians(90))
                .strafeToLinearHeading(secondInitialPose.position, secondInitialPose.heading)
                .build();
        double boardX = 48;
        PoseMap originalDirection = pose -> new Pose2dDual<>(pose.position.x, pose.position.y, pose.heading);
        PoseMap oppositeDirection = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
        Action leftBoard = drive.actionBuilder(secondInitialPose, isRed ? originalDirection : oppositeDirection)
                .strafeToLinearHeading(new Vector2d(boardX, -27), Math.toRadians(180))
                .build();
        Action middleBoard = drive.actionBuilder(secondInitialPose, isRed ? originalDirection : oppositeDirection)
                .strafeToLinearHeading(new Vector2d(boardX, -34), Math.toRadians(180))
                .build();
        Action rightBoard = drive.actionBuilder(secondInitialPose, isRed ? originalDirection : oppositeDirection)
                .strafeToLinearHeading(new Vector2d(boardX, -41), Math.toRadians(180))
                .build();

        waitForStart();
        lift.toggleClawOpen();

        float startTime = System.currentTimeMillis() / 1000f;
        int detection = detector.run(() -> {
            int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
                    (time > 2 ? "." : "") +
                    (time > 3 ? "." : ""));
            telemetry.update();
        });

        switch (detection) {
            case 0:
                Actions.runBlocking(left);
                if(!isRed) {
                    drive.pose = BuildersKt.map(oppositeDirection, drive.pose);
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
                    Actions.runBlocking(leftBoard);
                } else {
                    Actions.runBlocking(rightBoard);
                }
                break;
        }
        lift.setPosition(Lift.LiftPosition.Default.DOWN);
        while(!lift.getState().isFinished()) {
            lift.periodic();
        }
        lift.setPosition(Lift.LiftPosition.Default.LOW);
        while(!lift.getState().isFinished()) {
            lift.periodic();
        }
        lift.setPosition(Lift.LiftPosition.Default.DOWN);
        while(!lift.getState().isFinished()) {
            lift.periodic();
        }
    }
}
