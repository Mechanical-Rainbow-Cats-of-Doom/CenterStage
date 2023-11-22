package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

/**
 * This class is a proof of concept Path planner
 * It's called monkey because its supposed to be bad
 */
public class MonkeyPathPlanner implements PathPlanner {
    public static final String NAME = "Monkey";

    // constants for the movement controller
    public static double kpP = 0;
    public static double kiP = 0;
    public static double kdP = 0;
    private static PIDFController pathPID;

    // constants for the rotation controller
    public static double kpR = 0;
    public static double kiR = 0;
    public static double kdR = 0;
    private static PIDFController rotationPID;

    public MonkeyPathPlanner() {
        pathPID = new PIDFController(kpP, kiP, kdP, 0);
        rotationPID = new PIDFController(kpR, kiR, kdR, 0);
    }

    public String getName() {
        return NAME;
    }

    /**
     * Should be used for ftc dashboard and changing hte controllers on the spot
     */
    public void updateControllers() {
        pathPID.setPIDF(kpP, kiP, kdP, 0);
        rotationPID.setPIDF(kpR, kiR, kdR, 0);
    }

    /**
     * The goal of this path planner is to find the optimal unit vector from the current position to
     * the goal position, and then optimize the magnitude of the vector around the max speed
     * The hope is that it will slow down accurately to the goal point
     * @param currentVelocity THIS IS IN METERS PER SECOND! I do not believe this is necessary for a shitty pather
     * @param currentPose THIS IS IN INCHES!
     * @return A VELOCITY IN METERS PER SECOND FOR THE ROBOT TO DRIVE!
     */
    @Override
    public ChassisSpeeds calculateTargetVelocity(ChassisSpeeds currentVelocity, Pose2d currentPose, Path path) {
        /*
        This code gets the unit vector between the current position and the target point
         */
        double deltaX = path.getTargetPoint().getPose().minus(currentPose).getTranslation().getX(); // inches
        double deltaY = path.getTargetPoint().getPose().minus(currentPose).getTranslation().getY(); // inches
        double deltaMagnitude = Math.sqrt(Math.pow(deltaX, 2)+Math.pow(deltaY, 2)); // inches
        Vector2d unitVector = new Vector2d(deltaX/deltaMagnitude,deltaY/deltaMagnitude); // no unit, just proportions

        // terminate early if the distance is less than a set tolerance
        // the tolerance is set by the target point
        if (deltaMagnitude < path.getTargetPoint().getTolerance()) {
            path.incrementTargetPoint();
            return new ChassisSpeeds(0,0,0);
        }

        /*
        This code gets the magnitude of the vector between the current pose and the target
        in meters per second
        The first value is the measured distance between the points in inches
        The second value is the optimal distance between the points, always 0
         */
        double magnitude = pathPID.calculate(deltaMagnitude, 0); // mps

        /*
        this gets the optimal rotational velocity in radians per second
        The first value is the current rotation
        The second value is the target rotation
         */
        double rotationalVelocity = rotationPID.calculate(currentPose.getHeading(), path.getTargetPoint().getPose().getHeading()); // rps

        ChassisSpeeds targetVelocity = new ChassisSpeeds(unitVector.getX()*magnitude,
                unitVector.getY()*magnitude, rotationalVelocity); // be in meters per second
        return targetVelocity;
    }

}
