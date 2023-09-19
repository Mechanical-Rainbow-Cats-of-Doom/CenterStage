package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public abstract class ContinuousLocalization implements Localization {
    public Pose2d position;
    public int oldReadX, oldReadY;
    public double oldReadHeading;
    public final double xMultiplier, yMultiplier;
    public final double xOffset, yOffset;

    public ContinuousLocalization(Pose2d initialPosition, int initialReadX, int initialReadY,
                                  double xMultiplier, double yMultiplier, double xOffset,
                                  double yOffset) {
        this.xMultiplier = xMultiplier;
        this.yMultiplier = yMultiplier;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.oldReadX = initialReadX;
        this.oldReadY = initialReadY;
        this.oldReadHeading = initialPosition.getRotation().getRadians();
    }

    public ContinuousLocalization(Pose2d initialPosition, double xMultiplier, double yMultiplier, double xOffset,
                                  double yOffset) {
        this(initialPosition, 0, 0, xMultiplier, yMultiplier, xOffset, yOffset);
    }

    public void setPosition(Pose2d pose) {
        this.position = pose;
    }

    public void updatePosition(int xEncoderPos, int yEncoderPos, double heading) {
        double xDiff = (xEncoderPos - oldReadX) * xMultiplier;
        double yDiff = (yEncoderPos - oldReadY) * yMultiplier;

        xDiff -= xOffset * (heading-oldReadHeading);
        yDiff -= yOffset * (heading-oldReadHeading);

        double sinAverage = (Math.sin(heading)+Math.sin(oldReadHeading))/2;
        double cosAverage = (Math.cos(heading)+Math.cos(oldReadHeading))/2;

        double newX = xDiff * cosAverage + yDiff * sinAverage;
        double newY = xDiff * sinAverage + yDiff * cosAverage;

        position = new Pose2d(position.getX() + newX, position.getY() + newY,
                new Rotation2d(heading));

        oldReadX = xEncoderPos;
        oldReadY = yEncoderPos;
        oldReadHeading = heading;
    }

    @Override
    public Pose2d getPosition() {
        return position;
    }
}
