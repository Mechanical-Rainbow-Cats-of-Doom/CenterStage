package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

/**
 * Localization finds the current position based on an input.,
 *
 * @see ContinuousLocalization
 * @see DiscreteLocalization
 */
public interface Localization {
    Pose2d getPosition();

    ChassisSpeeds getVelocity();

//    void setPostition();

    void updatePosition();
}
