package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;

public class ControllerDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem drive;
    private final GamepadEx driveGamepad;
    private final BooleanSupplier presetOffsetToggle;

    public ControllerDriveCommand(SwerveDriveSubsystem drive, GamepadEx driveGamepad, BooleanSupplier presetOffsetToggle) {
        this.m_subsystem = "drive";
        this.drive = drive;
        this.driveGamepad = driveGamepad;
        this.presetOffsetToggle = presetOffsetToggle;
    }

    @Override
    public void execute() {
        drive.setUsePresetOffset(presetOffsetToggle.getAsBoolean());
        drive.setTargetVelocity(new ChassisSpeeds(driveGamepad.getLeftY(), driveGamepad.getLeftX(), driveGamepad.getRightX()));
    }
}
