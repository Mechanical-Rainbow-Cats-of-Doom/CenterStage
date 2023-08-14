package org.firstinspires.ftc.teamcode.drive.swerve;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import java.util.function.Supplier;

public class DefaultSwerveDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem drive;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    public DefaultSwerveDriveCommand(SwerveDriveSubsystem drive, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.drive = drive;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setTargetVelocity(chassisSpeedsSupplier.get());
    }

}
