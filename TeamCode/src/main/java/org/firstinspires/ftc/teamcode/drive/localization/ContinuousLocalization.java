package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

/**
 * ContinuousLocalization is the type of localization used by encoders. It starts at a set position,
 * finds the distance moved by the robot, and puts the robot there. It's continuous because it must
 * be run as often as possible to have an accurate estimate.
 *
 * @see DiscreteLocalization
 * @see Localization
 * @see Localization2EImpl
 */
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
        double deltaX = (xEncoderPos - oldReadX) * xMultiplier;
        double deltaY = (yEncoderPos - oldReadY) * yMultiplier;

        deltaX -= xOffset * (heading-oldReadHeading);
        deltaY -= yOffset * (heading-oldReadHeading);

        double avgHeading = (heading+oldReadHeading)/2;
        double deltaHeading = heading - oldReadHeading;

        if(deltaHeading != 0) {
            // TODO implement math in whiteboard channel
        }
        double sinAverage = Math.sin(avgHeading);
        double cosAverage = Math.cos(avgHeading);

        double newX = deltaX * cosAverage + deltaY * sinAverage;
        double newY = deltaX * sinAverage + deltaY * cosAverage;

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
