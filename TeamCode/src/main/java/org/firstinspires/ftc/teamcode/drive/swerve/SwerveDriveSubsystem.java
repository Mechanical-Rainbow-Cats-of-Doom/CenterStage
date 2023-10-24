package org.firstinspires.ftc.teamcode.drive.swerve;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;

import java.util.function.BooleanSupplier;

@Config
public class SwerveDriveSubsystem extends SubsystemBase implements HolonomicDrive {
    private static final double MAX_XY_VELOCITY = 1e+10; // temp value, i/s
    private static final double MAX_ROTATIONAL_VELOCITY = 1e+10 * Math.PI; // temp value, radians/s

    public static double flP = 1.7, flI = 0.2, flD = 0.7, frP = 0.6, frI = 0, frD = 0.1, blP = 0.6, blI = 0, blD = 0.1, brP = 0.6, brI = 0, brD = 0.1;
    private static double[][] pidConstants;
    private final IMU imu;
    private final ToggleButtonReader fieldRelativeReader;
    private final SwerveModule[] swerveModules;
    private final Telemetry telemetry;

    private boolean driveAsPercentage;
    private boolean fieldRelative;

    private static void fillPIDConstants() {
        SwerveDriveSubsystem.pidConstants = new double[][] {{flP, flI, flD}, {frP, frI, frD}, {blP, blI, blD}, {brP, brI, brD}};
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.14785, 0.13459),
            new Translation2d(0.14785, -0.13459),
            new Translation2d(-0.14785, 0.13459),
            new Translation2d(-0.14785, -0.13459)
    );

    /**
     * Constructor for Swerve Drive
     * @param hMap hardware map instance
     * @param driveAsPercentage if true, drives as percentage. if false, drives with m/s.
     *                          likely want to use true for driver controlled and false for auto
     */
    public SwerveDriveSubsystem(final HardwareMap hMap, final Telemetry telemetry, final boolean driveAsPercentage, final BooleanSupplier fieldRelativeButton) {
        final SwerveModule frontL = new SwerveModule(hMap,"frontLeftMotor", "frontLeftServo", "frontLeftEncoder", SwerveModule.Wheel.FRONT_LEFT);
        final SwerveModule frontR = new SwerveModule(hMap,"frontRightMotor", "frontRightServo", "frontRightEncoder", SwerveModule.Wheel.FRONT_RIGHT);
        final SwerveModule backL = new SwerveModule(hMap,"backLeftMotor", "backLeftServo", "backLeftEncoder", SwerveModule.Wheel.BACK_LEFT);
        final SwerveModule backR = new SwerveModule(hMap,"backRightMotor", "backRightServo", "backRightEncoder", SwerveModule.Wheel.BACK_RIGHT);
        this.swerveModules = new SwerveModule[] {frontL, frontR, backL, backR};
        this.imu = hMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
        this.telemetry = telemetry;
        this.driveAsPercentage = driveAsPercentage;
        this.fieldRelativeReader = new ToggleButtonReader(fieldRelativeButton);
        fillPIDConstants();
    }

    //THIS METHOD BAD
    @NonNull
    public static ChassisSpeeds chassisSpeedFromDriveSpeedPercentages(double xPercent, double yPercent, double rotationPercent) {
        final double xyAngle = Math.atan2(xPercent, yPercent);
        return new ChassisSpeeds(
                Math.sin(xyAngle) * MAX_XY_VELOCITY,
                Math.cos(xyAngle) * MAX_XY_VELOCITY,
                rotationPercent * MAX_ROTATIONAL_VELOCITY // i'm not sure about this, might have to change the math
        );
    }

    //THIS METHOD ALSO BAD, WANT TO HAVE TWO DIFFERENT DRIVE MODES, SET/LOCK HEADING WITH RIGHT STICK AND AMPLITUDE WITH LEFT STICK, AND WHAT I'M CURRENTLY IMPLEMENTING RIGHT NOW FOR AUTO. ANOTHER COMMAND MAYBE?
    @Override
    public void setTargetVelocity(ChassisSpeeds chassisSpeeds) {
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    chassisSpeeds.omegaRadiansPerSecond,
                    new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))
            );
        }
        final SwerveModuleState[] moduleStates;
        if (driveAsPercentage) {
            chassisSpeeds.omegaRadiansPerSecond *= Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0.05 ? 0 : Math.PI * -2D;
            moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1D); // TODO Does this work for 1.0?
        } else {
            throw new UnsupportedOperationException("Not implemented");
        }
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setTargetModuleState(moduleStates[i]);
        }
    }

    @Override
    public ChassisSpeeds getMeasuredVelocity() {
        return null;
    }

    @Override
    public Pose2d getPosition() {
        return null;
    }

    @Override
    public void periodic() {
        fieldRelativeReader.readValue();
        if (fieldRelativeReader.wasJustReleased()) {
            fieldRelative = !fieldRelative;
        }
        fillPIDConstants();
        for (int i = 0, swerveModulesLength = swerveModules.length; i < swerveModulesLength; i++) {
            SwerveModule module = swerveModules[i];
            module.read();
            module.update(pidConstants[i][0], pidConstants[i][1], pidConstants[i][2]);
            module.runTelemetry(Integer.toString(i), telemetry);
        }
        //telemetry.update();
    }

    public boolean isDrivingAsPercentage() {
        return driveAsPercentage;
    }

    public void setDriveAsPercentage(boolean driveAsPercentage) {
        this.driveAsPercentage = driveAsPercentage;
    }
}
