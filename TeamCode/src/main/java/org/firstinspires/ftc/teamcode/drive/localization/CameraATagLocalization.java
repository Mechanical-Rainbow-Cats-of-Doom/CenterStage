package org.firstinspires.ftc.teamcode.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import javax.annotation.Nullable;

/**
 * CameraATagLocalization is a form of (non-continuous) Localization which uses AprilTags to
 * find the current position.
 */
public class CameraATagLocalization implements DiscreteLocalization {
    public final AprilTagProcessor processor;
    public final Pose2d[] featureList;
    public Pose2d lastReadPosition = null;
    public long lastReadTime;

    /**
     * Creates a new AprilTag localizer. List length MUST be
     */
    public CameraATagLocalization(AprilTagProcessor processor, Pose2d[] featureList) {
        this.processor = processor;
        this.featureList = featureList;
        if(featureList.length != 587) {
            throw new RuntimeException("ERROR! featureList input into CameraATagLocalization MUST " +
                    "have 587 elements!");
        }
    }

    /**
     * Adds a new AprilTag feature with an ID and a pose.
     * @param id The ID of the AprilTag.
     * @param pose The pose of the AprilTag as a {@link Pose2d}.
     */
    public void setFeature(int id, Pose2d pose) {
        featureList[id] = pose;
    }

    /**
     * Returns the previously read position and the time it was read.
     */
    @Override @Nullable
    public Pose2d getPosition() {
        return lastReadPosition;
    }

    @Override
    public long getLastReadTime() {
        return lastReadTime;
    }

    @Override
    public void updatePosition() {
        boolean positionUpdated = false;
        for (AprilTagDetection detection : processor.getDetections()) {
            if(detection.id < 0 || detection.id > 586) {
                continue;
            }
            // TODO: Pose math.
        }
        if(positionUpdated) {
            lastReadTime = System.currentTimeMillis();
        }
    }
}
