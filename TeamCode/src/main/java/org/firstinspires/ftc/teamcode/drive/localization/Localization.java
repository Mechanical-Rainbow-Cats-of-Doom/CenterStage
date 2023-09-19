package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.jetbrains.annotations.TestOnly;

public interface Localization {
    Pose2d getPosition();

    void updatePosition();
}
