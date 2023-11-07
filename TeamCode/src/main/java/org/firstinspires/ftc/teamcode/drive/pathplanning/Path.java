package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.ArrayList;

public class Path {
    private ArrayList<Pose2d> poseArray;
    private Pose2d startingPose;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private int targetPoseIdx;

    public Path() {
        poseArray = new ArrayList<Pose2d>();
        startingPose = new Pose2d(0,0, new Rotation2d(0));
        currentPose = startingPose;
        targetPose = startingPose;
        targetPoseIdx = -1;
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }
    public Pose2d getStartingPose() {
        return startingPose;
    }

    public void setStartingPose(Pose2d startingPose) {
        this.startingPose = startingPose;
        if (poseArray.size() == 0) {
            currentPose = startingPose;
            targetPose = startingPose;
        }
    }

    public Path addPose(Pose2d newPose2d) {
        poseArray.add(newPose2d);
        return this;
    }

    public Path addPose(double x, double y, Rotation2d rotation) {
        poseArray.add(new Pose2d(x, y, rotation));
        return this;
    }

    public Path addRotation(Rotation2d rotation) {
        poseArray.add(new Pose2d(0, 0, rotation));
        return this;
    }

    public void advancePoints() {
        if(targetPoseIdx+1 > poseArray.size()) {
            return;
        }
        if(targetPoseIdx+1 == poseArray.size()) {
            targetPoseIdx++;
            currentPose = targetPose;
            return;
        }
        targetPoseIdx++;
        currentPose = targetPose;
        targetPose = poseArray.get(targetPoseIdx);
    }

    public void retreatPoints() {
        if (targetPoseIdx-1 < -1) {
            return;
        }
        if (targetPoseIdx-1 == -1) {
            targetPoseIdx--;

        }
        if(targetPoseIdx-1 == 0) {
            targetPoseIdx--;
            targetPose = currentPose;
            currentPose = startingPose;
        }
        targetPoseIdx--;
        targetPose = currentPose;
        currentPose = poseArray.get(targetPoseIdx-1);
    }

}
