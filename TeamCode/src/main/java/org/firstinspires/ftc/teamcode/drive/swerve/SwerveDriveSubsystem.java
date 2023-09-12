package org.firstinspires.ftc.teamcode.drive.swerve;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;

public class SwerveDriveSubsystem extends SubsystemBase implements HolonomicDrive {
    private static final double MAX_XY_VELOCITY = 1e+10; // temp value, m/s
    private static final double MAX_ROTATIONAL_VELOCITY = 1e+10 * Math.PI; // temp value, radians/s
    
    private final SwerveModule frontL, frontR, backL, backR;
    private final SwerveModule[] swerveModules;
    private boolean driveAsPercentage;

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
    public SwerveDriveSubsystem(final HardwareMap hMap, boolean driveAsPercentage) {
        this.frontL = new SwerveModule(hMap,"frontRightMotor", "frontRightServo", "frontRightEncoder");
        this.frontR = new SwerveModule(hMap,"frontRightMotor", "frontRightServo", "frontRightEncoder");
        this.backL = new SwerveModule(hMap,"backLeftMotor", "backLeftServo", "backLeftEncoder");
        this.backR = new SwerveModule(hMap,"backRightMotor", "backRightServo", "backRightEncoder");
        this.swerveModules = new SwerveModule[] {frontL, frontR, backL, backR};
        this.driveAsPercentage = driveAsPercentage;
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
        if (driveAsPercentage) {
            final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.0D); // TODO Does this work for 1.0?

        } else {
            // TODO write code for m/s control with PID Controller
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
        for (SwerveModule module : swerveModules) {
            module.read();
            module.update();
        }
    }

    public boolean isDrivingAsPercentage() {
        return driveAsPercentage;
    }

    public void setDriveAsPercentage(boolean driveAsPercentage) {
        this.driveAsPercentage = driveAsPercentage;
    }
}
