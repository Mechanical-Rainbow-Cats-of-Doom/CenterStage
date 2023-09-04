package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Localization2EImpl implements Localization {
    public Pose2d position;
    public DcMotorImpl xEncoder, yEncoder;
    public Rotation2d storedRotation;
    public IMU imu;
    public int oldReadX, oldReadY;
    public double oldReadHeading;

    // TODO tune these values
    public static final class Constants {
        public static final double xMultiplier = 1, yMultiplier = 1;
        public static final double xEncoderOffset = 0, yEncoderOffset = 0;
    }

    public Localization2EImpl(HardwareMap map, String xEncoder, String yEncoder, Pose2d startingPosition) {
        this.xEncoder = map.get(DcMotorImpl.class, xEncoder);
        this.yEncoder = map.get(DcMotorImpl.class, yEncoder);
        this.imu = map.get(IMU.class, "imu");
        oldReadX = this.xEncoder.getCurrentPosition();
        oldReadY = this.yEncoder.getCurrentPosition();
        position = startingPosition;
        storedRotation = startingPosition.getRotation();
        imu.resetYaw();
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
        int xInt = xEncoder.getCurrentPosition();
        int yInt = yEncoder.getCurrentPosition();

        // get position
        double xDiff = (xInt - oldReadX) * Constants.xMultiplier;
        double yDiff = (yInt - oldReadY) * Constants.yMultiplier;

        double heading = getYaw();
        double averageHeading = (heading + oldReadHeading) / 2;

        // TODO correct xDiff and yDiff for offset blah blah blah rotation arc length, image i posted on 9/3/23 at 8:56
        //  in my garbage zone


        double sinHeading = Math.sin(averageHeading);
        double cosHeading = Math.cos(averageHeading);

        double newX = xDiff * cosHeading + yDiff * sinHeading;
        double newY = xDiff * sinHeading + yDiff * cosHeading;

        position = new Pose2d(position.getX() + newX, position.getY() + newY,
                new Rotation2d(storedRotation.getRadians() + heading));

        oldReadX = xInt;
        oldReadY = yInt;
        oldReadHeading = heading;
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
