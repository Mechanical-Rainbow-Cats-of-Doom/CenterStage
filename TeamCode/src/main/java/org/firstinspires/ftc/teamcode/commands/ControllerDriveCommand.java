package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

public class ControllerDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem drive;
    private final GamepadEx driveGamepad;
    public ControllerDriveCommand(SwerveDriveSubsystem drive, GamepadEx driveGamepad) {
        this.m_subsystem = "drive";
        this.drive = drive;
        this.driveGamepad = driveGamepad;
    }

    @Override
    public void execute() {
        drive.setTargetVelocity(new ChassisSpeeds(driveGamepad.getLeftY(), driveGamepad.getLeftX(), driveGamepad.getRightX()));
    }
}
