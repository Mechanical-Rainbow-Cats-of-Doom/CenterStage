package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.geometry.Pose2d;

/**
 * Immutable, keep immutable
 */
public class Point {
    private Pose2d pose;
    private double tolerance;

    public Point(Pose2d pose, double tolerance) {
        this.pose = pose;
        this.tolerance = tolerance;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTolerance() {
        return tolerance;
    }

    /**
     * Overirde this in other impelementations of point with the nescessary information and the name of the point type
     * @return name of the point type, x, y, θ, inches tolerance
     */
    @Override
    public String toString() {
        return "Point: " + pose.getX() + "x, " + pose.getY() + "y, " + pose.getHeading() + "θ, " + tolerance + " inches tolerance";
    }
}
