package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public interface HolonomicDrive extends Subsystem {
    void setTargetVelocity(ChassisSpeeds targetVelocity);
}
