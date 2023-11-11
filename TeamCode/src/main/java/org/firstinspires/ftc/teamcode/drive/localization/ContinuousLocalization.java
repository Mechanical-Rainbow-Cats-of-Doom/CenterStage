package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

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
    public Rotation2d oldRotation;
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
        this.oldRotation = initialPosition.getRotation();
        this.position = initialPosition;
    }

    public ContinuousLocalization(Pose2d initialPosition, double xMultiplier, double yMultiplier, double xOffset,
                                  double yOffset) {
        this(initialPosition, 0, 0, xMultiplier, yMultiplier, xOffset, yOffset);
    }

    public void setPosition(Pose2d pose) {
        this.position = pose;
    }

    public Transform2d updatePosition(int xEncoderPos, int yEncoderPos, Rotation2d rotation) {
        double deltaX = (xEncoderPos - oldReadX) * xMultiplier;
        double deltaY = (yEncoderPos - oldReadY) * yMultiplier;

        double heading = rotation.getRadians();
        double oldHeading = oldRotation.getRadians();
        double minHeading = Math.min(heading, oldHeading);
        double maxHeading = Math.max(heading, oldHeading);

        deltaX -= xOffset * (heading - oldHeading);
        deltaY -= yOffset * (heading - oldHeading);

        double rawDeltaHeading = Math.abs(maxHeading - minHeading);
        // normalize between 0 and 2pi
        rawDeltaHeading -= 2*Math.PI * Math.floor(rawDeltaHeading/(2*Math.PI));
        // We don't know what direction we rotated in, but this makes a good guess!
        // if the angle is larger than pi, chances are we're going in the wrong direction!
        // edge case: if rawDeltaHeading is Math.PI, we would prefer to keep that. this edge case
        // should never happen in the real world but this made my unit tests nicer.
        double deltaHeading = (rawDeltaHeading == Math.PI) ? Math.PI :
                rawDeltaHeading - (Math.PI * Math.floor(rawDeltaHeading/Math.PI));
        double avgHeading = (rawDeltaHeading > Math.PI) ? maxHeading + (deltaHeading/2) :
                minHeading + (deltaHeading/2);
        // normalize between 0 and 2pi
        avgHeading -= 2*Math.PI*Math.floor(avgHeading/(2*Math.PI));

        if(deltaHeading != 0) {
            double rotationFactor = 2*Math.sin(deltaHeading/2);
            deltaX *= rotationFactor * deltaX/deltaHeading;
            deltaY *= rotationFactor * deltaY/deltaHeading;
        }
        double sinAverage = Math.sin(avgHeading);
        double cosAverage = Math.cos(avgHeading);

        double newX = deltaX * cosAverage + deltaY * sinAverage;
        double newY = deltaX * sinAverage + deltaY * cosAverage;

        position = new Pose2d(position.getX() + newX, position.getY() + newY, rotation);

        Transform2d transform = position.minus(new Pose2d(new Translation2d(oldReadX, oldReadY), oldRotation));

        oldReadX = xEncoderPos;
        oldReadY = yEncoderPos;
        oldRotation = rotation;

        return transform;
    }

    public abstract ChassisSpeeds getVelocity();

    @Override
    public Pose2d getPosition() {
        return position;
    }
}
