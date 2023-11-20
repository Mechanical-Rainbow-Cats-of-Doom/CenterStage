package org.firstinspires.ftc.teamcode.drive.localization;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A basic form of ContinuousLocalization using two encoders and an IMU.
 *
 * @see Localization
 * @see ContinuousLocalization
 */
public class Localization2EImpl extends ContinuousLocalization {
    public ChassisSpeeds velocity;
    public DcMotor xEncoder, yEncoder;
    public Rotation2d storedRotation;
    public IMU imu;
    public long oldReadTime;

    // TODO tune these values
    @Config
    public static final class LocalizationConstants {
        public static final double xMultiplier = 0.1, yMultiplier = 0.1;
        public static final double xEncoderOffset = 0, yEncoderOffset = 0;
    }

    private static class LocalizationConstantProvider implements ContinuousLocalization.ConstantsProvider {
        @Override
        public double getXMultiplier() {
            return LocalizationConstants.xMultiplier;
        }

        @Override
        public double getYMultiplier() {
            return LocalizationConstants.yMultiplier;
        }

        @Override
        public double getXOffset() {
            return LocalizationConstants.xEncoderOffset;
        }

        @Override
        public double getYOffset() {
            return LocalizationConstants.yEncoderOffset;
        }
    }

    public Localization2EImpl(DcMotor xEncoder, DcMotor yEncoder, IMU imu, Pose2d startingPosition,
                              LocalizationConstantProvider constantProvider) {
        super(startingPosition, xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(),
                constantProvider);
        oldReadTime = System.currentTimeMillis();
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        this.imu = imu;
        position = startingPosition;
        storedRotation = startingPosition.getRotation();
        if(imu != null)
            imu.resetYaw();
    }

    public Localization2EImpl(DcMotor xEncoder, DcMotor yEncoder, IMU imu, Pose2d startingPosition) {
        this(xEncoder, yEncoder, imu, startingPosition, new LocalizationConstantProvider());
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

    public int getXEncoderTicks() {
        return xEncoder.getCurrentPosition();
    }

    public int getYEncoderTicks() {
        return yEncoder.getCurrentPosition();
    }

    @Override
    public void setPosition(Pose2d position) {
        this.position = position;
        storedRotation = new Rotation2d(-getYaw().getRadians() + position.getHeading());
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    @Override
    public void updatePosition() {
        long time = System.currentTimeMillis();
        double deltaTime = (time - oldReadTime)/1000d;

        Transform2d transform = super.updatePosition(xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(), getYaw());
        velocity = new ChassisSpeeds(transform.getTranslation().getX() * deltaTime / 39.37,
                transform.getTranslation().getY() * deltaTime / 39.37,
                transform.getRotation().getRadians() * deltaTime);

        oldReadTime = time;
    }

    private Rotation2d getYaw() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + storedRotation.getRadians());
    }
}
