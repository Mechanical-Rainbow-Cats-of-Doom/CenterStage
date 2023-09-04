package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;

public interface Localization {
    Pose2d getPosition();

    void setPosition(Pose2d pose);

    void updatePosition();
}
