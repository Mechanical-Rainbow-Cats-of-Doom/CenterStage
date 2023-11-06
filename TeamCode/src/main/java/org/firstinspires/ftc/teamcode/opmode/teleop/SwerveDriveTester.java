package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
@Config
public class SwerveDriveTester extends LinearOpMode {
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    @Override
    public void runOpMode() {
        final GamepadEx driver1 = new GamepadEx(gamepad1), driver2 = new GamepadEx(gamepad2);
        final SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true, () -> driver1.getButton(GamepadKeys.Button.B));
        waitForStart();
        while (opModeIsActive()) {
            drive.setTargetVelocity(new ChassisSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightY())); // this is concerning if this works, y should not be left and right on gamepad
            drive.periodic();

            telemetry.addData("gamepad1 y", driver1.getLeftY());
            telemetry.addData("gamepad1 x", driver1.getLeftX());
            telemetry.addData("gamepad1 rightx", driver1.getRightX());
            telemetry.update();
        }
    }

}
