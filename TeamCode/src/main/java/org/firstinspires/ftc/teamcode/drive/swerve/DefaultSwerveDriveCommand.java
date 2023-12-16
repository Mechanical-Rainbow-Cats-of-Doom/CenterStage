package org.firstinspires.ftc.teamcode.drive.swerve;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DefaultSwerveDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem drive;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private final ToggleButtonReader presetOffsetToggle;

    public DefaultSwerveDriveCommand(SwerveDriveSubsystem drive, Supplier<ChassisSpeeds> chassisSpeedsSupplier, BooleanSupplier presetOffsetToggle) {
        this.drive = drive;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.presetOffsetToggle = new ToggleButtonReader(presetOffsetToggle);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        presetOffsetToggle.readValue();
        drive.setUsePresetOffset(presetOffsetToggle.getState());
        drive.setTargetVelocity(chassisSpeedsSupplier.get());
    }

}
