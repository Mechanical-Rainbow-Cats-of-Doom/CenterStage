package org.firstinspires.ftc.teamcode.opmode.auto.newautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class boardAlley extends LinearOpMode {
    public static final Pose2d startPose = new Pose2d(12, -61.75, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

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

        Action cycle = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(35, -12))
                .strafeToLinearHeading(new Vector2d(-57, -12), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -12), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(50, -35), Math.toRadians(180))
                .build();



        int visionOutput = -1;

        waitForStart();
        if (isStopRequested()) return;

        Action direction;
        switch (visionOutput) {
            case 0:
                direction = left;
                break;
            case 1:
                direction = center;
                break;
            case 2:
                direction = right;
                break;
            default:
                direction = center;
                break;
        }



    }
}
