package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Localization2EImpl implements Localization {
    public Pose2d position;
    public DcMotor xEncoder, yEncoder;
    public Rotation2d storedRotation;
    public IMU imu;
    public int oldReadX, oldReadY;
    public double oldReadHeading;

    // TODO tune these values
    public static final class Constants {
        public static final double xMultiplier = 1, yMultiplier = 1;
        public static final double xEncoderXOffset = 0, xEncoderYOffset = 0,
                yEncoderXOffset = 0, yEncoderYOffset = 0;
        public static boolean xUpdated = true, yUpdated = true;
        private static double xDistance = 0, yDistance = 0;

        public static double xDistance() {
            if(xUpdated) {
                xDistance = Math.sqrt(xEncoderXOffset*xEncoderXOffset+xEncoderYOffset*xEncoderYOffset);
                xUpdated = false;
            }
            return xDistance;
        }

        public static double yDistance() {
            if(yUpdated) {
                yDistance = Math.sqrt(yEncoderXOffset*yEncoderXOffset+yEncoderYOffset*yEncoderYOffset);
                yUpdated = false;
            }
            return yDistance;
        }
    }

    public Localization2EImpl(DcMotor xEncoder, DcMotor yEncoder, IMU imu, Pose2d startingPosition) {
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        this.imu = imu;
        oldReadX = this.xEncoder.getCurrentPosition();
        oldReadY = this.yEncoder.getCurrentPosition();
        position = startingPosition;
        storedRotation = startingPosition.getRotation();
        imu.resetYaw();
    }

    public Localization2EImpl(HardwareMap map, String xEncoder, String yEncoder, Pose2d startingPosition) {
        this(map.get(DcMotor.class, xEncoder), map.get(DcMotor.class, yEncoder),
                map.get(IMU.class, "imu"), startingPosition);
    }

    public Localization2EImpl(HardwareMap map, String xEncoder, String yEncoder) {
        this(map, xEncoder, yEncoder, new Pose2d());
    }

    @Override
    public Pose2d getPosition() {
        return position;
    }

    @Override
    public void setPosition(Pose2d position) {
        this.position = position;
        storedRotation = new Rotation2d(-getYaw() + position.getHeading());
    }

    @Override
    public void updatePosition() {
        updatePosition(xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(), getYaw());
    }

    @Override
    public void updatePosition(int xEncoderPos, int yEncoderPos, double heading) {
        double xDiff = (xEncoderPos - oldReadX) * Constants.xMultiplier;
        double yDiff = (yEncoderPos - oldReadY) * Constants.yMultiplier;

        double averageHeading = (heading + oldReadHeading) / 2;

        xDiff -= Constants.xDistance()*(heading-oldReadHeading);
        yDiff -= Constants.yDistance()*(heading-oldReadHeading);

        double sinHeading = Math.sin(averageHeading);
        double cosHeading = Math.cos(averageHeading);

        double newX = xDiff * cosHeading + yDiff * sinHeading;
        double newY = xDiff * sinHeading + yDiff * cosHeading;

        position = new Pose2d(position.getX() + newX, position.getY() + newY,
                new Rotation2d(heading));

        oldReadX = xEncoderPos;
        oldReadY = yEncoderPos;
        oldReadHeading = heading;
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + storedRotation.getRadians();
    }
}
