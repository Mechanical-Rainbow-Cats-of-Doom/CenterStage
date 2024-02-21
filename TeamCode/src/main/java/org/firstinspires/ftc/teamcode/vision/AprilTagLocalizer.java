package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.opmode.config.aprilTagTester;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalizer {

    // vision processors
    private AprilTagProcessor aprilProcessor;
    private VisionPortal visionPortal;

    /**
     * The tolerance of the point validation. Measured in inches.
     */
    private final static double TOLERANCE = 5;

    // Camera position controls
    private final static float CAMERA_X = 10f;
    private final static float CAMERA_Y = 10f;
    private final static float CAMERA_Z = 10f;
    /**
     * Location of the Camera on the robot
     */
    private final static VectorF CAMERA_LOCATION = new VectorF(CAMERA_X, CAMERA_Y, CAMERA_Z);

    /**
     * Initializes the April Tag Processor and the Vision Portal
     */
    public AprilTagLocalizer() {
        initializeAprilTagProcessor();
        initializeVisionPortal(aprilProcessor);
    }

    /**
     * Initializes the localizer with a preset vision portal and aprilProcessor
     */
    public AprilTagLocalizer(VisionPortal visionPortal, AprilTagProcessor aprilProcessor) {
        this.visionPortal = visionPortal;
        this.aprilProcessor = aprilProcessor;
    }

    /**
     * Private method to initialize the April Tag processor
     */
    private void initializeAprilTagProcessor() {
        // initialize builder
        AprilTagProcessor.Builder aprilBuilder = new AprilTagProcessor.Builder();

        aprilBuilder.setTagLibrary(getCenterStageTagLibrary());

        // custom features, more available
        aprilBuilder.setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false);

        // initialize processor with the builder
        aprilProcessor = aprilBuilder.build();
    }

    /**
     * Private method to initialize the vision portal
     */
    private void initializeVisionPortal(AprilTagProcessor aprilProcessor) {
        // initialize vision portal builder
        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();

        // set camera
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        // set the processor
        visionPortalBuilder.addProcessor(aprilProcessor);

        // custom features, more available
        visionPortalBuilder.setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2) // MJPEG uses less bandwidth
                .enableLiveView(false) // Enable LiveView (RC preview).
                .setAutoStopLiveView(true);

        // initialize the vision portal
        visionPortal = visionPortalBuilder.build();
    }

    /**
     * Relatively fast. Disabling the processor will save resources
     */
    public void setProcessorState(boolean running) {
        visionPortal.setProcessorEnabled(aprilProcessor, running);
    }

    /**
     * Uses All the Found April Tags to Estimate your Position.
     * Find all april tags.
     * Validate the positions found.
     * Return an average.
     *
     * @return position on the field or null if we are unable to find it
     */
    public VectorF getPositionVector() {
        List<AprilTagDetection> aprilDetections = aprilProcessor.getDetections();
        if (aprilDetections.size() >= 2) {
            boolean valid = true;
            for (AprilTagDetection aprilTagDetection : aprilDetections) {
                if (aprilTagDetection.metadata == null) valid = false;
            }
            if (valid) {
                // todo: make a case for when both angles are equal
                boolean firstDetectionLarger = aprilDetections.get(0).ftcPose.bearing > aprilDetections.get(1).ftcPose.bearing;
                AprilTag left = new AprilTag(firstDetectionLarger ? aprilDetections.get(0) : aprilDetections.get(1));
                AprilTag right = new AprilTag(firstDetectionLarger ? aprilDetections.get(1) : aprilDetections.get(0));

                double angleDifference;
                if (left.bearing >= 0 && right.bearing >= 0) {
                    angleDifference = Math.abs(left.bearing - right.bearing);
                } else if (left.bearing < 0 && right.bearing < 0) {
                    angleDifference = Math.abs(Math.abs(left.bearing) - Math.abs(right.bearing));
                } else {
                    angleDifference = Math.abs(left.bearing) + Math.abs(right.bearing);
                }
                double connectingLength = lawOfCos(left.hyp, right.hyp, angleDifference);
                VectorF connectingVector = new VectorF((float) (connectingLength * Math.cos(angleDifference)), (float) (connectingLength * Math.sin(angleDifference)));

                //angle 1
                double leftAngle = lawOfSin(connectingLength, right.hyp, angleDifference);
                double rightAngle = lawOfSin(connectingLength, left.hyp, angleDifference);
                if (Math.abs(Math.PI - (rightAngle + leftAngle + angleDifference)) > TOLERANCE) {
                    if (left.hyp > right.hyp) {
                        rightAngle = Math.PI - rightAngle;
                    }
                    if (right.hyp > left.hyp) {
                        leftAngle = Math.PI - leftAngle;
                    }
                }

                // Adjust the angles to work with the vectors
                double adjustedLeftAngle = (3 * Math.PI) / 2 - leftAngle;
                double adjustedRightAngle = Math.PI / 2 + rightAngle;

                boolean flipX = left.detection.metadata.fieldPosition.get(0) < 0;
                VectorF leftLocation = left.detection.metadata.fieldPosition
                        .added(new VectorF(
                                (float) (left.hyp * Math.cos(adjustedLeftAngle)),
                                (float) (left.hyp * Math.sin(adjustedLeftAngle)),
                                (float) (left.detection.ftcPose.z)
                        ).multiplied(flipX ? -1 : 1));
                VectorF rightLocation = right.detection.metadata.fieldPosition
                        .added(new VectorF(
                                (float) (right.hyp * Math.cos(adjustedRightAngle)),
                                (float) (right.hyp * Math.sin(adjustedRightAngle)),
                                (float) (right.detection.ftcPose.z)
                        ).multiplied(flipX ? -1 : 1));

                return leftLocation;
            }
        }
        return null;
    }

    /**
     * Average all positions given with equal weight.
     *
     * @param positions all valid positions
     * @return an average of all positions
     */
    private static VectorF equalWeightAverage(List<VectorF> positions) {
        VectorF sumVector = new VectorF(0, 0, 0);
        for (VectorF position : positions) {
            sumVector.add(position);
        }
        return sumVector.multiplied(1f / (float) positions.size());
    }

    private class AprilTag {
        public AprilTagDetection detection;
        public final double hyp, bearing, yaw;

        public AprilTag(AprilTagDetection detection) {
            this.detection = detection;
            this.hyp = Math.sqrt(Math.pow(detection.ftcPose.x, 2) + Math.pow(detection.ftcPose.y, 2));
            this.bearing = Math.toRadians(detection.ftcPose.bearing);
            this.yaw = Math.toRadians(detection.ftcPose.yaw);
        }
    }

    /**
     * Law of Cosines. c^2 = a^2 + b^2 - 2ab*cos(C)
     * lowercase are sides, uppercase are angles
     *
     * @param a length of the first side
     * @param b length of the second side
     * @param C measure of the angle opposite to the third side, radians
     * @return length of the third side
     */
    private static double lawOfCos(double a, double b, double C) {
        return Math.sqrt(
                Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(C)
        );
    }

    /**
     * Law of Sines. A/sin(a) = B/sin(b)
     * lowercase are sides, uppercase are angles
     *
     * @param a length of the first side
     * @param b length of the second side
     * @param A measure of the angle opposite to the first side, radians
     * @return measure of the angle opposite to the second side
     */
    private static double lawOfSin(double a, double b, double A) {
        return Math.asin(
                (b * Math.sin(A)) / a
        );
    }
}
