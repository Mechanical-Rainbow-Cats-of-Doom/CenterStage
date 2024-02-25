package org.firstinspires.ftc.teamcode.opmode.auto.newautos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@Autonomous
public class audienceAlley extends LinearOpMode {
    public static final Pose2d startPose = new Pose2d(12, -61.75, Math.toRadians(90));
    public static boolean isRed = true;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize camera
        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                isRed, PropPipeline.PropPipelineRectsProvider.Default.RED_BOARD_SIDE);

        NewLift lift = new NewLift(hardwareMap);
        NewIntake intake = new NewIntake(hardwareMap);

        drive = new MecanumDrive(hardwareMap, startPose);
        int yMultiplier = isRed ? 1 : -1;

        // Purple pixel movements
        Action left = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(12, -45*yMultiplier), Math.toRadians(110))
                .strafeToLinearHeading(new Vector2d(6, -40*yMultiplier), Math.toRadians(130))
                .strafeToLinearHeading(new Vector2d(50, -35*yMultiplier), Math.toRadians(180))
                .build();

        Action center = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, -32*yMultiplier))
                .strafeTo(new Vector2d(20, -35*yMultiplier))
                .strafeToLinearHeading(new Vector2d(50, -35*yMultiplier), Math.toRadians(180))
                .build();

        Action right = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(18, -40*yMultiplier), Math.toRadians(70))
                .strafeToLinearHeading(new Vector2d(30, -45*yMultiplier), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -35*yMultiplier), Math.toRadians(180))
                .build();

        Action cycle = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(35, -12))
                .strafeToLinearHeading(new Vector2d(-57, -12*yMultiplier), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -12*yMultiplier), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -35*yMultiplier), Math.toRadians(180))
                .build();

//        Action purpleMovement = drive.actionBuilder()

        int visionOutput = -1;

        waitForStart();
        if (isStopRequested()) return;

        // TODO:
        float startTime = System.currentTimeMillis() / 1000f;
        int detection = detector.run(() -> {
            int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
                    (time > 2 ? "." : "") +
                    (time > 3 ? "." : ""));
            telemetry.update();
        });

        Action purplePixel;
        switch (visionOutput) {
            case 0:
                purplePixel = left;
                break;
            case 1:
                purplePixel = center;
                break;
            case 2:
                purplePixel = right;
                break;
            default:
                purplePixel = center;
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
