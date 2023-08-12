package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public interface HolonomicDrive {
    void setTargetVelocity(ChassisSpeeds targetVelocity);
    ChassisSpeeds getMeasuredVelocity();
    Pose2d getPosition();
}
