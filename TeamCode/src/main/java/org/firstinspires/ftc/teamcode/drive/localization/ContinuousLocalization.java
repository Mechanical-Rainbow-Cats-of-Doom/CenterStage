package org.firstinspires.ftc.teamcode.drive.localization;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
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
    protected Pose2d position;
    protected int oldReadX, oldReadY;
    protected Rotation2d oldRotation;
    protected final ConstantsProvider constantsProvider;
    protected double Xrobot, Yrobot;
    protected ChassisSpeeds velocity;

    public interface ConstantsProvider {
        double getXMultiplier();
        double getYMultiplier();
        double getXOffset();
        double getYOffset();
    }

    public ContinuousLocalization(Pose2d initialPosition, int initialReadX, int initialReadY,
                                  ConstantsProvider constantsProvider) {
        this.constantsProvider = constantsProvider;
        this.oldReadX = initialReadX;
        this.oldReadY = initialReadY;
        this.oldRotation = initialPosition.getRotation();
        this.position = initialPosition;
    }

    public ContinuousLocalization(Pose2d initialPosition, ConstantsProvider constantsProvider) {
        this(initialPosition, 0, 0, constantsProvider);
    }

    public void setPosition(Pose2d pose) {
        this.position = pose;
    }

    public void setRobotCoordinates(Translation2d robotCoordinates) {
        Xrobot = robotCoordinates.getX();
        Yrobot = robotCoordinates.getY();
    }

    public Translation2d getRobotCoordinates() {
        return new Translation2d(Xrobot, Yrobot);
    }

    public void updatePosition(int xEncoderPos, int yEncoderPos, @NonNull Rotation2d rotation,
                               double xEncoderVelocity, double yEncoderVelocity,
                               float angularVelocity) {
        double deltaX = (xEncoderPos - oldReadX) * constantsProvider.getXMultiplier();
        double deltaY = (yEncoderPos - oldReadY) * constantsProvider.getYMultiplier();
        double xVelocity = xEncoderVelocity * constantsProvider.getXMultiplier();
        double yVelocity = yEncoderVelocity * constantsProvider.getYMultiplier();

        // position
        double heading = rotation.getRadians();
        double oldHeading = oldRotation.getRadians();
        double minHeading = Math.min(heading, oldHeading);
        double maxHeading = Math.max(heading, oldHeading);

        // position
        deltaX -= constantsProvider.getXOffset() * (heading - oldHeading);
        deltaY -= constantsProvider.getYOffset() * (heading - oldHeading);
        // velocity
        xVelocity -= constantsProvider.getXOffset() * angularVelocity;
        yVelocity -= constantsProvider.getYOffset() * angularVelocity;

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

//        if(deltaHeading != 0) {
//            double rotationFactor = 2*Math.sin(deltaHeading/2);
//            deltaX *= rotationFactor * deltaX/deltaHeading;
//            deltaY *= rotationFactor * deltaY/deltaHeading;
//        }
        Xrobot += deltaX;
        Yrobot += deltaY;

        double sinAverage = Math.sin(avgHeading);
        double cosAverage = Math.cos(avgHeading);

        double newX = deltaX * cosAverage + deltaY * sinAverage;
        double newY = deltaX * sinAverage + deltaY * cosAverage;
        double velX = xVelocity * cosAverage + yVelocity * sinAverage;
        double velY = xVelocity * sinAverage + yVelocity * cosAverage;

        position = new Pose2d(position.getX() + newX, position.getY() + newY, rotation);
        velocity = new ChassisSpeeds(velX/39.3701, velY/39.3701,
                angularVelocity);

        oldReadX = xEncoderPos;
        oldReadY = yEncoderPos;
        oldRotation = rotation;
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    @Override
    public Pose2d getPosition() {
        return position;
    }
}
