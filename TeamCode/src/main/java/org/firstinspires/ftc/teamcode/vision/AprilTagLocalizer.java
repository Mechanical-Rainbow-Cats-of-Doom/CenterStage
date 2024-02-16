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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalizer {

    // Debug variables
    private boolean debug = false;
    private Telemetry telemetry;

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
        initializeVisionPortal();
    }

    /**
     * Initializes the April Tag Processor and the Vision Portal.
     * Including Telemtery enables debug mode.
     */
    public AprilTagLocalizer(Telemetry telemetry) {
        this();
        debug = true;
        this.telemetry = telemetry;
    }

    /**
     * Private method to initialize the April Tag processor
     */
    private void initializeAprilTagProcessor() {
        // initialize builder
        AprilTagProcessor.Builder aprilBuilder = new AprilTagProcessor.Builder();

        aprilBuilder.setTagLibrary(getCenterStageTagLibrary());

        // custom features, more available
        aprilBuilder.setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true);

        // initialize processor with the builder
        aprilProcessor = aprilBuilder.build();
    }

    /**
     * Private method to initialize the vision portal
     */
    private void initializeVisionPortal() {
        // initialize vision portal builder
        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();

        // set camera
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        // set the processor
        visionPortalBuilder.addProcessor(aprilProcessor);

        // custom features, more available
        // TODO: put in the correct resolution for our camera
        visionPortalBuilder.setCameraResolution(new Size(640, 480)).setStreamFormat(VisionPortal.StreamFormat.YUY2) // MJPEG uses less bandwidth
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
    public VectorF getPosition(Vector2d currentPosition) {
        /*
        Here are the axis designations in the new SDK:

            Y axis points straight outward from the camera lens center

            X axis points to the right (looking outward), perpendicular to the Y axis

            Z axis points upward, perpendicular to Y and X
         */

        List<AprilTagDetection> aprilDetections = aprilProcessor.getDetections();
        // if there are no detections then return null
        if (aprilDetections == null) return null;

        // get the position on the field as told by each april tag
        List<VectorF> positions = new ArrayList<>();
        for (AprilTagDetection detection : aprilDetections) {
            // position relative to the april tag
            VectorF detectionPosition = new VectorF((float) detection.ftcPose.x, (float) detection.ftcPose.y, (float) detection.ftcPose.z);
            /*
                Add the position of the April tag,
                the position relative to the april tag,
                and the position of the camera relative to the center of the robot
             */
            positions.add(detectionPosition.added(detection.metadata.fieldPosition).added(CAMERA_LOCATION));
        }

        // replace the positions list with a new list of valid positions
        positions = validatePoseMeasurements(positions);
        // if there are no valid positions then return null
        if (positions == null) return null;


        // return the average of all the valid vectors
        return equalWeightAverage(positions);
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

    /**
     * Validates that measured pose from april tags is valid.
     * Must see more than one tag.
     * If there are two found tags then validate those two.
     * If three are found then compare each of those to each other
     * If at least one pair is valid then continue.
     *
     * @param positions all positions from the detected april tags
     * @return all valid measurements, null if there are none
     */
    private static List<VectorF> validatePoseMeasurements(List<VectorF> positions) {
        // not valid if there is only one detected
        if (positions.size() == 1) return null;

        // create the list of valid detections
        List<VectorF> validDetections = new ArrayList<>();

        // case of two measurements
        if (positions.size() == 2) {
            // compare item 0 and item 1
            if (validateLocationsByMagnitudeOfDifference(positions.get(0), positions.get(1))) {
                validDetections.add(positions.get(0));
                validDetections.add(positions.get(1));
                return validDetections;
            }
            // if they all fail then return null
            return null;
        }

        // compare combos of the first three tags if there are more than 2
        boolean compare0and1 = validateLocationsByMagnitudeOfDifference(positions.get(0), positions.get(1));
        boolean compare0and2 = validateLocationsByMagnitudeOfDifference(positions.get(0), positions.get(2));
        boolean compare1and2 = validateLocationsByMagnitudeOfDifference(positions.get(1), positions.get(2));
        // return them all if all combos pass
        if (compare0and1 && compare1and2 && compare0and2) {
            validDetections.add(positions.get(0));
            validDetections.add(positions.get(1));
            validDetections.add(positions.get(2));
            return validDetections;
        }
        // return the first individual combo
        if (compare0and1) {
            validDetections.add(positions.get(0));
            validDetections.add(positions.get(1));
            return validDetections;
        }
        if (compare0and2) {
            validDetections.add(positions.get(0));
            validDetections.add(positions.get(2));
            return validDetections;
        }
        if (compare1and2) {
            validDetections.add(positions.get(1));
            validDetections.add(positions.get(2));
            return validDetections;
        }
        // if they all fail then return null
        return null;
    }

    /**
     * First find the vector between the points.
     * Then find the magnitude of that vector.
     * THen compare the magnitude to the tolerance (m<T).
     *
     * @param vec1 the first vector to compare against
     * @param vec2 the second vector to compare against
     * @return true if the located position of both are within tolerance
     */
    private static boolean validateLocationsByMagnitudeOfDifference(VectorF vec1, VectorF vec2) {
        VectorF difV = vec1.subtracted(vec2);
        float difM = difV.magnitude();
        return difM < TOLERANCE;
    }

    /**
     * Convert a roadrunner pose2d to an FTC vectorF
     *
     * @param pose a pose2d describing the position of the robot
     * @return a new vectorF
     */
    private static VectorF pose2dToVectorF(Pose2d pose) {
        return new VectorF((float) pose.position.x, (float) pose.position.y);
    }

    /**
     * Obtain the VectorF from an April Tag detection
     *
     * @param detection the april tag you detected
     * @return a new vectorF
     */
    private static VectorF aprilTagToVectorF(AprilTagDetection detection) {
        return new VectorF((float) detection.ftcPose.x, (float) detection.ftcPose.y, (float) detection.ftcPose.z);
    }
}
