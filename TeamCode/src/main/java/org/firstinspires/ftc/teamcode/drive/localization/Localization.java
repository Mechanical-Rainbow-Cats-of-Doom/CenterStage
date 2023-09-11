package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;

public interface Localization {
    Pose2d getPosition();

    void setPosition(Pose2d pose);

    void updatePosition();

    @Deprecated
    void updatePosition(int xEncoderPos, int yEncoderPos, double imuRotation);
}
