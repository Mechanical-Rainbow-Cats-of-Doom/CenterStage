package org.firstinspires.ftc.teamcode.drive.localization;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * A basic form of ContinuousLocalization using two encoders and an IMU.
 *
 * @see Localization
 * @see ContinuousLocalization
 */
public class Localization2EImpl extends ContinuousLocalization {
    protected DcMotorEx xEncoder, yEncoder;
    protected Rotation2d storedRotation;
    protected IMU imu;
    protected int initialXPosition, initialYPosition;
    protected long lastCountRunTime;
    protected long runCounter;

    // TODO tune these values
    @Config
    public static final class LocalizationConstants {
        public static double xMultiplier = 0.0029250457038391, yMultiplier = 0.0029268651639582;
        public static double xEncoderOffset = 6.375, yEncoderOffset = 0.125;
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

    public Localization2EImpl(DcMotorEx xEncoder, DcMotorEx yEncoder, IMU imu, Pose2d startingPosition,
                              LocalizationConstantProvider constantProvider) {
        super(startingPosition, xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(),
                constantProvider);
        initialXPosition = xEncoder.getCurrentPosition();
        initialYPosition = yEncoder.getCurrentPosition();
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        this.imu = imu;
        position = startingPosition;
        storedRotation = startingPosition.getRotation();
        if(imu != null)
            imu.resetYaw();
    }

    public Localization2EImpl(DcMotorEx xEncoder, DcMotorEx yEncoder, IMU imu, Pose2d startingPosition) {
        this(xEncoder, yEncoder, imu, startingPosition, new LocalizationConstantProvider());
    }

    public Localization2EImpl(HardwareMap map, String xEncoder, String yEncoder, Pose2d startingPosition) {
        this(map.get(DcMotorEx.class, xEncoder), map.get(DcMotorEx.class, yEncoder),
                map.get(IMU.class, "imu"), startingPosition);
    }

    public Localization2EImpl(HardwareMap map, String xEncoder, String yEncoder) {
        this(map, xEncoder, yEncoder, new Pose2d());
    }

    public Localization2EImpl(HardwareMap map) {
        this(map, EncoderNames.xEncoder, EncoderNames.yEncoder);
    }

    @Override
    public Pose2d getPosition() {
        return position;
    }

    public int getXEncoderTicks() {
        return xEncoder.getCurrentPosition() - initialXPosition;
    }

    public int getYEncoderTicks() {
        return yEncoder.getCurrentPosition() - initialYPosition;
    }

    @Override
    public void setPosition(Pose2d position) {
        super.setPosition(position);
        storedRotation = new Rotation2d(-getYaw().getRadians() - storedRotation.getRadians() + position.getRotation().getRadians());
    }

    public long currentRunCountsTime() {
        return System.currentTimeMillis() - lastCountRunTime;
    }

    public double getRunFrequency() {
        long time = System.currentTimeMillis();
        double runFrequency = runCounter/((time - lastCountRunTime)/1000d);
        runCounter = 0;
        lastCountRunTime = time;
        return runFrequency;
    }

    @Override
    public void updatePosition() {
        runCounter += 1;

        super.updatePosition(xEncoder.getCurrentPosition(), yEncoder.getCurrentPosition(), getYaw(),
                xEncoder.getVelocity(), yEncoder.getVelocity(),
                imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);
    }

    private Rotation2d getYaw() {
        return new Rotation2d(
                imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle +
                        storedRotation.getRadians());
    }
}
