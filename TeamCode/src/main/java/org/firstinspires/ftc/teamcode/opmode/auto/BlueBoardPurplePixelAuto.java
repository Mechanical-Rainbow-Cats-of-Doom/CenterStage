package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.util.DelayStorage;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@Autonomous
public class BlueBoardPurplePixelAuto extends LinearOpMode {
    protected boolean isRed = false;
    protected boolean closeRightTurn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                isRed, PropPipeline.PropPipelineRectsProvider.Default.RED_BOARD_SIDE);
        Intake intake = new Intake(hardwareMap);
        Pose2d startPose = new Pose2d(12, -61.75, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action left = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(12,-45), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6, -40), Math.toRadians(130))
                .strafeToLinearHeading(new Vector2d(12,-45), Math.toRadians(90))
                .strafeTo(new Vector2d(12, -50))
                .build();
        Action middle = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(12, -33))
                .strafeTo(new Vector2d(12, -50))
                .build();
        Action right = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(12,-41), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(closeRightTurn ? 45 : 35))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(12,-41), Math.toRadians(90))
                .strafeTo(new Vector2d(12, -50))
                .build();
        Timing.Timer timer = DelayStorage.getTimer();
        waitForStart();
        timer.start();

        float startTime = System.currentTimeMillis() / 1000f;
        int detection = detector.run(() -> {
            int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
                    (time > 2 ? "." : "") +
                    (time > 3 ? "." : ""));
            telemetry.update();
        });

        while(!timer.done()) {
            telemetry.addLine("Waiting for delay");
            telemetry.addData("Time Left", timer.remainingTime() / 1000);
            telemetry.update();
        }
        switch (detection) {
            case 0:
                Actions.runBlocking(left);
                break;
            case 1:
                Actions.runBlocking(middle);
                break;
            case 2:
                Actions.runBlocking(right);
                break;
        }
    }
}
