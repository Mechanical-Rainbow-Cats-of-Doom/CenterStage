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
    public final double xMultiplier, yMultiplier;
    public final double xEncoderOffset, yEncoderOffset;

    // TODO tune these values
    public static final class Constants {
        public static final double xMultiplier = 0.1, yMultiplier = 0.1;
        public static final double xEncoderOffset = 0, yEncoderOffset = 0;
    }

    public Localization2EImpl(DcMotor xEncoder, DcMotor yEncoder, IMU imu, Pose2d startingPosition,
                              double xMultiplier, double yMultiplier, double xEncoderOffset,
                              double yEncoderOffset) {
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        this.imu = imu;
        if(xEncoder != null)
            oldReadX = this.xEncoder.getCurrentPosition();
        if(yEncoder != null)
            oldReadY = this.yEncoder.getCurrentPosition();
        position = startingPosition;
        storedRotation = startingPosition.getRotation();
        oldReadHeading = startingPosition.getRotation().getRadians();
        if(imu != null)
            imu.resetYaw();
        this.xMultiplier = xMultiplier;
        this.yMultiplier = yMultiplier;
        this.xEncoderOffset = xEncoderOffset;
        this.yEncoderOffset = yEncoderOffset;
    }

    public Localization2EImpl(Pose2d startingPosition, double xMultiplier, double yMultiplier,
                              double xEncoderOffset, double yEncoderOffset) {
        this(null, null, null, startingPosition, xMultiplier, yMultiplier,
                xEncoderOffset, yEncoderOffset);
    }

    public Localization2EImpl(DcMotor xEncoder, DcMotor yEncoder, IMU imu, Pose2d startingPosition) {
        this(xEncoder, yEncoder, imu, startingPosition, Constants.xMultiplier,
                Constants.yMultiplier, Constants.xEncoderOffset, Constants.yEncoderOffset);
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
        internalUpdatePosition(xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(), getYaw());
    }

    public void updatePosition(int xEncoderPos, int yEncoderPos, double heading) {
        internalUpdatePosition(xEncoderPos, yEncoderPos, heading);
    }

    public void internalUpdatePosition(int xEncoderPos, int yEncoderPos, double heading) {
        double xDiff = (xEncoderPos - oldReadX) * xMultiplier;
        double yDiff = (yEncoderPos - oldReadY) * yMultiplier;

        double averageHeading = (heading + oldReadHeading) / 2;

        xDiff -= xEncoderOffset*(heading-oldReadHeading);
        yDiff -= yEncoderOffset*(heading-oldReadHeading);

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
