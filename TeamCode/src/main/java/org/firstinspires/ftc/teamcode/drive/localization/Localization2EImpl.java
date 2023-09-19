package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Localization2EImpl extends ContinuousLocalization {
    public Pose2d position;
    public DcMotor xEncoder, yEncoder;
    public Rotation2d storedRotation;
    public IMU imu;

    // TODO tune these values
    public static final class Constants {
        public static final double xMultiplier = 0.1, yMultiplier = 0.1;
        public static final double xEncoderOffset = 0, yEncoderOffset = 0;
    }

    public Localization2EImpl(DcMotor xEncoder, DcMotor yEncoder, IMU imu, Pose2d startingPosition,
                              double xMultiplier, double yMultiplier, double xEncoderOffset,
                              double yEncoderOffset) {
        super(startingPosition, xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(),
                xMultiplier, yMultiplier, xEncoderOffset, yEncoderOffset);
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        this.imu = imu;
        position = startingPosition;
        storedRotation = startingPosition.getRotation();
        if(imu != null)
            imu.resetYaw();
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
        updatePosition(xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(), getYaw());
    }

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + storedRotation.getRadians();
    }
}
