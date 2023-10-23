package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
@Config
public class SwerveDriveFromConfig extends LinearOpMode {
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);

    public static double driverY, driverLX, driverRX;
    @Override
    public void runOpMode() {
        final SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true);
        waitForStart();
        while (opModeIsActive()) {
            drive.setTargetVelocity(new ChassisSpeeds(driverY, driverLX, driverRX));
            drive.periodic();
            telemetry.update();
        }
    }
}
