package org.firstinspires.ftc.teamcode.opmode.config;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp
public class aprilTagTester extends LinearOpMode {
    public static boolean SETPOSITION = false;
    public static boolean CLEARPOSITION = false;
    public static double TOLERANCE = Math.toRadians(2);
//    public static boolean USE_HEADING = false;
//    public static int HEADING_MULTI = 1;
//    public static boolean USE_BEARING = false;
//    public static int BEARING_MULTI = 1;
//    public static boolean USE_YAW = false;
//    public static int YAW_MULTI = 1;
//    public static int X_MULTIPLIER = 1;
//    public static int Y_MULTIPLIER = 1;
//    public static boolean SWAP_AXIS = false;

    IMU imu;
    AprilTagProcessor.Builder aprilBuilder;
    AprilTagProcessor aprilProcessor;
    VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;
    MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    FtcDashboard dashboard = FtcDashboard.getInstance();

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
        double val = Math.sqrt(
                Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(C)
        );
//        return Double.isNaN(val) ? 0 : val;
        return val;
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
        double val = Math.asin(
                (b * Math.sin(A)) / a
        );
        return Double.isNaN(val) ? 0 : val;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -61.75, Math.toRadians(90)));

        imu = hardwareMap.get(IMU.class, "imu");

        // initialize builder
        aprilBuilder = new AprilTagProcessor.Builder();

        aprilBuilder.setTagLibrary(getCenterStageTagLibrary());

        // custom features, more available
        aprilBuilder.setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true);

        // initialize processor with the builder
        aprilProcessor = aprilBuilder.build();

        // initialize vision portal builder
        visionPortalBuilder = new VisionPortal.Builder();

        // set camera
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        // set the processor
        visionPortalBuilder.addProcessor(aprilProcessor);

        // custom features, more available
        // TODO: put in the correct resolution for our camera
        visionPortalBuilder.setCameraResolution(new Size(1920, 1080)).setStreamFormat(VisionPortal.StreamFormat.YUY2) // MJPEG uses less bandwidth
                .enableLiveView(true) // Enable LiveView (RC preview).
                .setAutoStopLiveView(true);

        // initialize the vision portal
        visionPortal = visionPortalBuilder.build();


        // setting up for detection
        List<AprilTagDetection> aprilDetections;  // list of all detections
        int aprilDetectionID = -145;                           // ID code of current detection, in for() loop
        int fill = 0;

        waitForStart();

        /*
        Here are the axis designations in the new SDK:

            Y axis points straight outward from the camera lens center

            X axis points to the right (looking outward), perpendicular to the Y axis

            Z axis points upward, perpendicular to Y and X
         */
        List<Double> rotations = new ArrayList<>();
        List<VectorF> positions = new ArrayList<>();
        List<Double> xVals = new ArrayList<>();
        List<Double> yVals = new ArrayList<>();

        while (opModeIsActive() && !isStopRequested()) {
            rotations.clear();
            TelemetryPacket packet = new TelemetryPacket();

            if (CLEARPOSITION) {
                positions.clear();
                CLEARPOSITION = false;
            }

            telemetry.addData("x values", xVals.toString());
            telemetry.addData("y values", yVals.toString());

//            packet.fieldOverlay().setFill("green").fillCircle(0, 0, 2);
//            packet.fieldOverlay().setFill("purple").fillCircle(10, 0, 2);
//            packet.fieldOverlay().setFill("green").fillCircle(0, 10, 2);

            // Get a list of AprilTag detections.
            aprilDetections = aprilProcessor.getDetections();

            if (aprilDetections.size() >= 2) {
                boolean valid = true;
                for (AprilTagDetection aprilTagDetection : aprilDetections) {
                    if (aprilTagDetection.metadata == null) valid = false;
                }
                if (valid) {
                    // todo: make a case for when both angles are equal
                    boolean firstDetectionLarger = aprilDetections.get(0).ftcPose.bearing > aprilDetections.get(1).ftcPose.bearing;
                    AprilTag left = new AprilTag(firstDetectionLarger ? aprilDetections.get(0) : aprilDetections.get(1));
//                    telemetry.addData("t1", left.detection.id);
                    AprilTag right = new AprilTag(firstDetectionLarger ? aprilDetections.get(1) : aprilDetections.get(0));
//                    telemetry.addData("t2", right.detection.id);

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
//                    telemetry.addData("Difference to 180", Math.toDegrees(Math.abs(Math.PI - (rightAngle + leftAngle + angleDifference))));
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

                    // telemetry
//                    telemetry.addData("left angle", Math.toDegrees(leftAngle));
//                    packet.fieldOverlay().setFill("fuchsia")
//                            .fillCircle(left.hyp * Math.cos(leftAngle), left.hyp * Math.sin(leftAngle), 2);
//                    telemetry.addData("left angle adjusted", Math.toDegrees(adjustedLeftAngle));
//                    telemetry.addData("right angle", Math.toDegrees(rightAngle));
//                    packet.fieldOverlay().setFill("green")
//                            .fillCircle(right.hyp * Math.cos(rightAngle), right.hyp * Math.sin(rightAngle), 2);
//                    telemetry.addData("right angle adjusted", Math.toDegrees(adjustedRightAngle));
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

                    packet.fieldOverlay()
                            .setStrokeWidth(2)
                            .setFill("plum")
                            .setStroke("plum")
//                            .fillCircle(left.hyp * Math.cos(adjustedLeftAngle), left.hyp * Math.sin(adjustedLeftAngle), 2)
                            .strokeLine(left.detection.metadata.fieldPosition.get(0), left.detection.metadata.fieldPosition.get(1), leftLocation.get(0), leftLocation.get(1))
                            .setFill("chartreuse")
                            .setStroke("chartreuse")
//                            .fillCircle(right.hyp * Math.cos(adjustedRightAngle), right.hyp * Math.sin(adjustedRightAngle), 2)
                            .strokeLine(right.detection.metadata.fieldPosition.get(0), right.detection.metadata.fieldPosition.get(1), rightLocation.get(0), rightLocation.get(1));

                    telemetry
                            .addData("Right x", rightLocation.get(0));
                    telemetry
                            .addData("Right y", rightLocation.get(1));
                    telemetry.addData("Right distance", right.hyp);


                    telemetry
                            .addData("Left x", leftLocation.get(0));
                    telemetry
                            .addData("Left y", leftLocation.get(1));
                    telemetry.addData("Left distance", left.hyp);

                    if (SETPOSITION) {
                        positions.add(rightLocation);
                        xVals.add((double) rightLocation.get(0));
                        yVals.add((double) rightLocation.get(1));
                        SETPOSITION = false;
                    }


//                    telemetry.addData("t1 angle larger", firstDetectionLarger);
//                    telemetry.addData("location", rightLocation.toString());
//                    telemetry.addData("angle", adjustedRightAngle);
//                    telemetry.addData("connecting", connectingLength);
//                    telemetry.addData("angle diff", Math.toDegrees(angleDifference));
//                    packet.fieldOverlay().setFill("gold").fillCircle(connectingVector.get(0), connectingVector.get(1), 2);
                }
            }

            for (AprilTagDetection aprilDetection : aprilDetections) {

                if (aprilDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    aprilDetectionID = aprilDetection.id;

                    // Now take action based on this tag's ID code, or store info for later action.


                    VectorF distance = new VectorF((float) 1 * (float) (false ? aprilDetection.ftcPose.y : aprilDetection.ftcPose.x), (float) 1 * (float) (false ? aprilDetection.ftcPose.x : aprilDetection.ftcPose.y), (float) aprilDetection.ftcPose.z);
                    double angle = 2;
//                            (USE_HEADING ? HEADING_MULTI * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : 0) +
//                                    (USE_BEARING ? BEARING_MULTI * Math.toRadians(aprilDetection.ftcPose.bearing) : 0) +
//                                    (USE_YAW ? YAW_MULTI * Math.toRadians(aprilDetection.ftcPose.yaw) : 0);
                    VectorF rotateDistance = rotateVectorF(distance, angle);
                    VectorF location = aprilDetection.metadata.fieldPosition
                            .added(rotateDistance);

                    double x = aprilDetection.ftcPose.x;
                    double y = aprilDetection.ftcPose.y;
                    double bearing = Math.toRadians(aprilDetection.ftcPose.bearing);
                    double hypot = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

                    VectorF rotDistance = new VectorF(
                            (float) (hypot * Math.cos(bearing)),
                            (float) (hypot * Math.sin(bearing)),
                            (float) aprilDetection.ftcPose.z
                    );

                    switch (fill) {
                        case 0:
                            packet.fieldOverlay().setFill("red");
                            break;
                        case 1:
                            packet.fieldOverlay().setFill("orange");
                            break;
                        case 2:
                            packet.fieldOverlay().setFill("blue");
                            break;
                    }
                    packet.fieldOverlay().fillCircle(aprilDetection.metadata.fieldPosition.get(0), aprilDetection.metadata.fieldPosition.get(1), 2);
//                    packet.fieldOverlay().fillCircle(distance.get(0), distance.get(1), 2);
//                    packet.fieldOverlay().fillCircle(location.get(0), location.get(1), 2);
                    switch (fill) {
                        case 0:
                            packet.fieldOverlay().setFill("fuchsia");
                            fill = 1;
                            break;
                        case 1:
                            packet.fieldOverlay().setFill("yellow");
                            fill = 2;
                            break;
                        case 2:
                            packet.fieldOverlay().setFill("teal");
                            fill = 0;
                            break;
                    }
//                    packet.fieldOverlay().fillCircle(rotDistance.get(0), rotDistance.get(1), 2);
//                    packet.fieldOverlay().fillCircle(rotateDistance.get(0), rotateDistance.get(1), 2);
                    rotations.add(aprilDetection.ftcPose.yaw);
                }
            }

//            telemetry.addData("IMU YAW", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//            telemetry.addData("April YAW", rotations.toString());
            packet.fieldOverlay().setFill("brown").fillCircle(drive.pose.position.x, drive.pose.position.y, 3);
            for (VectorF position : positions) {
                packet.fieldOverlay().setFill("gold").fillCircle(position.get(0), position.get(1), 1);
            }
            dashboard.sendTelemetryPacket(packet);
//            telemetry.addData("Measuring", aprilDetectionID);
            telemetry.addLine(aprilDetections.toString());
            if (drive.pose != null) {
                telemetry.addData("Odo x", drive.pose.position.x);
                telemetry.addData("Odo y", drive.pose.position.y);
            }
            telemetry.update();
            drive.updatePoseEstimate();

        }
    }

    private static VectorF rotateVectorF(VectorF vector, Double angle) {
        double hyp = Math.sqrt(Math.pow(vector.get(0), 2) + Math.pow(vector.get(1), 2));
        return new VectorF(
                (float) (hyp * Math.cos(angle)),
                (float) (hyp * Math.sin(angle)),
                vector.get(2)
        );
    }
}
