package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.DefaultSwerveDriveCommand;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

// If you are confused how this works, please see TestAuto for more comments

@TeleOp
public class TestTeleOp extends CommandOpMode {
    private SwerveDriveSubsystem swerveDrive;
    private GamepadEx driver1, driver2;

    @Override
    public void initialize() {
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        swerveDrive = new SwerveDriveSubsystem(hardwareMap, telemetry, true);
        swerveDrive.setDefaultCommand(new PerpetualCommand(
                new DefaultSwerveDriveCommand(
                        swerveDrive,
                        () -> new ChassisSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX())
                )
        ));
    }
}
