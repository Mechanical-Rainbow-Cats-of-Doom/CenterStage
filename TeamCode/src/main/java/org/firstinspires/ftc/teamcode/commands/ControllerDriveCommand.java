package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;

public class ControllerDriveCommand extends CommandBase {
    private final HolonomicDrive drive;
    private final GamepadEx driveGamepad;
    private final BooleanSupplier presetOffsetToggle;

    public ControllerDriveCommand(HolonomicDrive drive, GamepadEx driveGamepad, BooleanSupplier presetOffsetToggle) {
        this.m_subsystem = "drive";
        this.drive = drive;
        this.driveGamepad = driveGamepad;
        this.presetOffsetToggle = presetOffsetToggle;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (drive instanceof SwerveDriveSubsystem) {
            final SwerveDriveSubsystem swerve = (SwerveDriveSubsystem) drive;
            swerve.setUsePresetOffset(presetOffsetToggle.getAsBoolean());
        }
        drive.setTargetVelocity(new ChassisSpeeds(driveGamepad.getLeftY(), driveGamepad.getLeftX(), driveGamepad.getRightX()));
    }
}
