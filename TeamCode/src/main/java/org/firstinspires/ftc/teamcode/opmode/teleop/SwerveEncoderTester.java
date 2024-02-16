package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveModule;

@Disabled
@Config
@TeleOp
public class SwerveEncoderTester extends LinearOpMode {
    final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    public static double RADIANS = 0D;
    @Override
    public void runOpMode() throws InterruptedException {
        final SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true, () -> false);
        waitForStart();
        drive.setPowerForAllPods(SwerveModule.EPSILON * 2);
        while (opModeIsActive()) {
            drive.setDirectionForAllPods(RADIANS);
            drive.periodic();
            telemetry.update();
        }
    }
}
