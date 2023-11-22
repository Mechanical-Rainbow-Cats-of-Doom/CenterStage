package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.ArrayList;

/**
 * Guide for proper use:
 * 1. Never append to a path while it is being run
 */
public class Path {
    private ArrayList<Point> pointList;
    private boolean pathFinished = false;
    private Pose2d startingPose;
    /**
     * Used for finding the next goal point
     * it will be -1 if the list is empty, then it will cause errors if people try to access the list
     */
    private int targetPoseIdx;
    /**
     * Not set yet, will need to be tuned currently 1e-3 as of this comment
     */
    public static final double DEFAULT_TOLERANCE = 1e-3; // inches

    public Path() {
        pointList = new ArrayList<Point>();
        targetPoseIdx = -1;
        startingPose = new Pose2d(0,0,new Rotation2d(0));
    }

    public Path(Pose2d startingPose) {
        pointList = new ArrayList<Point>();
        targetPoseIdx = -1;
        this.startingPose = startingPose;
    }

    public void setStartingPose(Pose2d startingPose) {
        this.startingPose = startingPose;
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }

    public int size() {
        return pointList.size();
    }

    public void resetTargetPoint() {
        if (targetPoseIdx != -1) {
            targetPoseIdx = 0;
        }
    }

    // theoretically people should only be able to access points through the target point, keep unimplemented
//    public Point get(int index) {
//        if (index < 0 || index > pointList.size()-1) {
//            throw new IndexOutOfBoundsException("Index is either less than 0 or greater than the size: " + pointList.size());
//        }
//        return pointList.get(index);
//    }

    public Point getTargetPoint() {
        if (targetPoseIdx == -1) {
            throw new IllegalStateException("Path is empty");
        }
        return pointList.get(targetPoseIdx);
    }

    public void incrementTargetPoint() {
        if (targetPoseIdx+1 >= pointList.size()) {
            pathFinished = true;
            return;
        }
        targetPoseIdx++;
    }

    public void decrementTargetPoint() {
        if (targetPoseIdx-1 < 0) {
            return;
        }
        pathFinished = false;
        targetPoseIdx--;
    }

    public boolean isPathFinished() {
        return pathFinished;
    }

    public Path removePose(int index) {
        if (index < 0 || index > pointList.size()-1) {
            throw new IndexOutOfBoundsException("Index is either less than 0 or greater than the size: " + pointList.size());
        }
        pointList.remove(index);
        if (pointList.size() == 0) {
            targetPoseIdx = -1;
        } else {
            targetPoseIdx = 0;
        }
        return this;
    }

    public void clear() {
        pointList.clear();
        targetPoseIdx = -1;
    }

    public Path appendPath(Path path) {
        for (int i = 0; i<path.size(); i++) {
            addPoint(path.getTargetPoint());
            path.incrementTargetPoint();
        }
        return this;
    }

    /*
    Various Ways to add individual points, choose your favorite!
     */
    public Path addPoint(Point newPoint) {
        pointList.add(newPoint);
        targetPoseIdx=0;
        return this;
    }

    public Path addPoint(Pose2d pose2d, double tolerance) {
        pointList.add(new Point(pose2d, tolerance));
        targetPoseIdx=0;
        return this;
    }

    public Path addPoint(Pose2d pose2d) {
        pointList.add(new Point(pose2d, DEFAULT_TOLERANCE));
        targetPoseIdx=0;
        return this;
    }

    public Path addPoint(double x, double y, double heading, double tolerance) {
        pointList.add(new Point(new Pose2d(x, y, new Rotation2d(heading)), tolerance));
        targetPoseIdx=0;
        return this;
    }

    public Path addPoint(double x, double y, double heading) {
        pointList.add(new Point(new Pose2d(x, y, new Rotation2d(heading)), DEFAULT_TOLERANCE));
        targetPoseIdx=0;
        return this;
    }

    public Path addPoint(double x, double y) {
        pointList.add(new Point(new Pose2d(x, y, new Rotation2d(0)), DEFAULT_TOLERANCE));
        targetPoseIdx=0;
        return this;
    }

    /**
     * Should be used for testing purposes
     * @return a string representation of the path
     */
    @Override
    public String toString() {
        String result = "";
        for (int i=0; i<pointList.size(); i++) {
            result += (pointList.get(i));
        }
        return result;
    }
}
