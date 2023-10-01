package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
public class SwerveDriveTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        final GamepadEx driver1 = new GamepadEx(gamepad1), driver2 = new GamepadEx(gamepad2);
        final SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true);
        waitForStart();
        while (opModeIsActive()) {
            drive.setTargetVelocity(new ChassisSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX()));
            drive.periodic();

            telemetry.addData("gamepad1 y", driver1.getLeftY());
            telemetry.addData("gamepad1 x", driver1.getLeftX());
            telemetry.addData("gamepad1 rightx", driver1.getRightX());
            telemetry.update();
        }

    }

}
