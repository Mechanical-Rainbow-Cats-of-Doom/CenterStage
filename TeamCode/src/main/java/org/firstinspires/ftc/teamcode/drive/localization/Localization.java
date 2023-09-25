package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;

/**
 * Localization finds the current position based on an input.,
 *
 * @see ContinuousLocalization
 * @see DiscreteLocalization
 */
public interface Localization {
    Pose2d getPosition();

    void updatePosition();
}
