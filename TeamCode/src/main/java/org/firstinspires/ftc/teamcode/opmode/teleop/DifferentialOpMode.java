package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@Disabled
@TeleOp
public class DifferentialOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        final GamepadEx gamepad = new GamepadEx(gamepad1);
        final DifferentialDrive drive = SwerveDriveSubsystem.getDifferentialSwerve(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drive.arcadeDrive(gamepad.getLeftY(), gamepad.getRightX(), true);
        }
    }
}
