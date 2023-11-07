package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

/**
 * This class is a proof of concept Path planner
 * It's called monkey because its supposed to be bad
 */
public class MonkeyPathPlanner implements PathPlanner {
    private Path path;
    private Pose2d currentPoint;
    private Pose2d targetPoint;
    public static double kpP = 0;
    public static double kiP = 0;
    public static double kdP = 0;
    private PIDFController pathPID;
    public static double kpR = 0;
    public static double kiR = 0;
    public static double kdR = 0;
    private PIDFController rotationPID;
    private static double tolerance = 1e-4;

    /*
    public MonkeyPathPlanner() {
        path = new Path();
        currentPoint = path.getStartingPose();
//        targetPoint = path.
        pathPID = new PIDFController(kp, ki, kd, 0);
    }
    */

    public MonkeyPathPlanner(Path path) {
        this.path = path;
        currentPoint = path.getStartingPose();
        pathPID = new PIDFController(kpP, kiP, kdP, 0);
        rotationPID = new PIDFController(kpR, kiR, kdR, 0);
    }

    public void resetPath() {
        currentPoint = path.getStartingPose();
    }

    /**
     * I HATE ETHAN AND FIN!!
     * WHY DO YOU GIVE ME MISMATCHING UNITS!
     * The goal of this path planner is to find the optimal unit vector from the current position to
     * the goal position, and then optimize the magnitude of the vector around the max speed
     * The hope is that it will slow down accurately to the goal point
     * @param currentVelocity THIS IS IN METERS PER SECOND! I do not believe this is necessary for a shitty pather
     * @param currentPose THIS IS IN INCHES!
     * @return A VELOCITY IN METERS PER SECOND FOR THE ROBOT TO DRIVE!
     */
    @Override
    public ChassisSpeeds calculateTargetVelocity(ChassisSpeeds currentVelocity, Pose2d currentPose) {
        /*
        This code gets the unit vector between the current position and the target point
         */
        double vX = targetPoint.minus(currentPose).getTranslation().getX();
        double vY = targetPoint.minus(currentPose).getTranslation().getY();
        double magnitude = Math.sqrt(Math.pow(vX, 2)+Math.pow(vY, 2));
        Vector2d unitVector = new Vector2d(vX/magnitude,vY/magnitude);

        /*
        This code gets the magnitude of the vector between the current pose and the target
        in meters per second
         */
        double maxV = SwerveDriveSubsystem.MAX_XY_VELOCITY;
        double maxRV = SwerveDriveSubsystem.MAX_ROTATIONAL_VELOCITY;



        ChassisSpeeds targetVelocity = new ChassisSpeeds();
        return null;
    }

    public static double inchToMeter(double inch) {
        return inch/39.37;
    }
}
