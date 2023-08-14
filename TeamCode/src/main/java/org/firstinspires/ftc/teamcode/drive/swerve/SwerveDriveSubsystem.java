package org.firstinspires.ftc.teamcode.drive.swerve;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.AutonomousHolonomicDrive;

//THIS SHOULD BE ABSTRACT MAYBE? IDK
public class SwerveDriveSubsystem extends SubsystemBase implements AutonomousHolonomicDrive {
    private static final double MAX_XY_VELOCITY = 1e+10; // temp value, m/s
    private static final double MAX_ROTATIONAL_VELOCITY = 1e+10 * Math.PI; // temp value, radians/s

    public SwerveDriveSubsystem(final HardwareMap hMap) {

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

    }
}
