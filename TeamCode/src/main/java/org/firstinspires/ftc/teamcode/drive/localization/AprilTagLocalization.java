package org.firstinspires.ftc.teamcode.drive.localization;

import android.util.Pair;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import javax.annotation.Nullable;

/**
 * AprilTagLocalization is a form of (non-continuous) Localization which uses AprilTags to
 * find the current position.
 *
 * Heed the deprecation mark below, this does not actually work
 */
public class AprilTagLocalization implements DiscreteLocalization {
    public final AprilTagProcessor processor;
    public Pose2d position = null;
    public Pose2d cameraOffset;
    public static final float minDetectDistance = 15;
    public long lastReadTime;
    public @Nullable Telemetry telemetry;
    public final AprilTagLibrary library;

    /**
     * Creates a new AprilTag localizer.
     */
    public AprilTagLocalization(AprilTagProcessor processor, Pose2d cameraOffset,
                                @Nullable Telemetry telemetry, AprilTagLibrary library) {
        this.processor = processor;
        this.cameraOffset = cameraOffset;
        this.telemetry = telemetry;
        this.library = library;
    }

    public AprilTagLocalization(AprilTagProcessor processor, Pose2d cameraOffset,
                                @Nullable Telemetry telemetry) {
        this(processor, cameraOffset, telemetry, AprilTagGameDatabase.getCenterStageTagLibrary());
    }

    public AprilTagLocalization(AprilTagProcessor processor, Pose2d cameraOffset) {
        this(processor, cameraOffset, null);
    }

    /**
     * Returns the previously read position and the time it was read.
     */
    @Override @Nullable
    public Pose2d getPosition() {
        return position;
    }

    @Override
    public long getLastReadTime() {
        return lastReadTime;
    }

    @Override
    public void updatePosition() {
        boolean positionUpdated = false;

        List<Pair<Pose2d, Double>> poses = new ArrayList<>();
        for (AprilTagDetection d : processor.getDetections()) {
            if(d.id < 0 || d.id > 586 || d.rawPose == null) continue;

            Pair<Pose2d, Double> poseEstimate = convertToPose2D(d, library.lookupTag(d.id));
            if(telemetry != null) {
                telemetry.addData("Distance " + d.id, poseEstimate.second);
                telemetry.addData("Est. Position" + d.id, poseEstimate.first.toString());
            }

            poses.add(poseEstimate);
        }
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

        position = new Pose2d(xPos, yPos, rotation);
        if(telemetry != null) {
            telemetry.addData("Estimated Position:", position);
        }

        if(positionUpdated) {
            lastReadTime = System.currentTimeMillis();
        }
    }

    /**
     * returns the estimated pose based on the detection,
     */
    public static Pair<Pose2d, Double> convertToPose2D(AprilTagDetection detection,
                                                       AprilTagMetadata metadata) {
        // TODO math
        return new Pair<>(new Pose2d(detection.ftcPose.x, detection.ftcPose.y,
                new Rotation2d(metadata.fieldOrientation.toOrientation(AxesReference.EXTRINSIC,
                        AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle -
                        detection.ftcPose.yaw)), detection.ftcPose.range);
    }
}
