package org.firstinspires.ftc.teamcode.drive.localization;

import android.util.Pair;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.stream.Collectors;

import javax.annotation.Nullable;

/**
 * CameraATagLocalization is a form of (non-continuous) Localization which uses AprilTags to
 * find the current position.
 */
public class CameraATagLocalization implements DiscreteLocalization {
    public final AprilTagProcessor processor;
    public final Pose2d[] featureList;
    public Pose2d lastReadPosition = null;
    public Pose2d cameraOffset;
    public static final float minDetectDistance = 15;
    public static final float multiplier = 1;
    public long lastReadTime;

    /**
     * Creates a new AprilTag localizer. List length MUST be
     */
    public CameraATagLocalization(AprilTagProcessor processor, Pose2d[] featureList,
                                  Pose2d cameraOffset) {
        this.processor = processor;
        this.featureList = featureList;
        if(featureList.length != 587) {
            throw new RuntimeException("ERROR! featureList input into CameraATagLocalization MUST " +
                    "have 587 elements!");
        }
        this.cameraOffset = cameraOffset;
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

        List<Pair<Pose2d, Double>> poses = processor.getDetections().stream()
                .filter((d) -> d.id > 0 && d.id < 586).map((d) -> estimatePose(d, featureList[d.id]))
                .collect(Collectors.toList());
        if(poses.size() == 0) return;
        double minDistance = poses.stream().mapToDouble((a) -> a.second).min().getAsDouble();
        if(minDistance < minDetectDistance) return;

        double distanceSum = poses.stream().mapToDouble((a) -> a.second).sum();

        double xPos = 0;
        double yPos = 0;
        Rotation2d rotation = poses.stream().reduce((a, b) -> (a.second < b.second) ? a : b).get().first.getRotation();
        for (Pair<Pose2d, Double> pair : poses) {
            xPos += pair.first.getX() * pair.second;
            yPos += pair.first.getY() * pair.second;
        }
        xPos /= distanceSum;
        yPos /= distanceSum;

        lastReadPosition = new Pose2d(xPos, yPos, rotation);

        if(positionUpdated) {
            lastReadTime = System.currentTimeMillis();
        }
    }

    /**
     * returns the estimated pose based on the detection,
     */
    public static Pair<Pose2d, Double> estimatePose(AprilTagDetection detection, Pose2d referencePose) {
        // TODO make this work
        final double twoSeventy = Math.toRadians(270);
        double distance = Math.hypot(detection.ftcPose.x, detection.ftcPose.y) * multiplier;

        Rotation2d vectorAngle = new Rotation2d(Math.atan(detection.ftcPose.y/detection.ftcPose.x));
        Rotation2d robotRotation = new Rotation2d(detection.ftcPose.pitch +
                referencePose.getHeading() - twoSeventy - vectorAngle.getRadians());

        Pose2d pose = new Pose2d(-distance * vectorAngle.getCos(),
                -distance * vectorAngle.getSin(), robotRotation);

        return new Pair<>(pose, distance);

    }
}
