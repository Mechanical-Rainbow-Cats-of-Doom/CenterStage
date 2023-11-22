package org.firstinspires.ftc.teamcode.drive.testingOPModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveController;
import org.firstinspires.ftc.teamcode.drive.pathplanning.MonkeyPathPlanner;
import org.firstinspires.ftc.teamcode.drive.pathplanning.Path;
import org.firstinspires.ftc.teamcode.drive.pathplanning.Point;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
public class BackAndForwardTest extends LinearOpMode {
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        DriveController<MonkeyPathPlanner, SwerveDriveSubsystem> drive =
                new DriveController<MonkeyPathPlanner, SwerveDriveSubsystem>(hardwareMap, new MonkeyPathPlanner(),
                        new SwerveDriveSubsystem(hardwareMap, telemetry, false, () -> false));

        Point startingPoint = new Point(new Pose2d(0,0, new Rotation2d(0)), Path.DEFAULT_TOLERANCE);
        Point endingPoint = new Point(new Pose2d(50,0, new Rotation2d(0)), Path.DEFAULT_TOLERANCE);

        Path forward = new Path().addPoint(endingPoint);
        Path backward = new Path().addPoint(startingPoint);

        // TODO: Add an override for correcting the robot position
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.followPath(forward);
            drive.followPath(backward);
        }
    }
}
