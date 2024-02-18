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
    public static int X_MULTIPLIER = 1;
    public static int Y_MULTIPLIER = 1;
    public static boolean SWAP_AXIS = false;

    IMU imu;
    AprilTagProcessor.Builder aprilBuilder;
    AprilTagProcessor aprilProcessor;
    VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;
    MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private class AprilTag {
        public AprilTagDetection detection;
        public final double hyp, bearing;

        public AprilTag(AprilTagDetection detection) {
            this.detection = detection;
            this.hyp = Math.sqrt(Math.pow(detection.ftcPose.x, 2) + Math.pow(detection.ftcPose.y, 2));
            this.bearing = Math.toRadians(detection.ftcPose.bearing);
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
                Math.pow(a, 2) + Math.pow(b, 2) - 4 * a * b * Math.cos(C)
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

        while (opModeIsActive() && !isStopRequested()) {
            rotations.clear();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setFill("green").fillCircle(0, 0, 2);
            packet.fieldOverlay().setFill("purple").fillCircle(10, 0, 2);
            packet.fieldOverlay().setFill("green").fillCircle(0, 10, 2);

            // Get a list of AprilTag detections.
            aprilDetections = aprilProcessor.getDetections();

            // Cycle through through the list and process each AprilTag.
            if (aprilDetections.size() >= 2) {
                boolean valid = true;
                for (AprilTagDetection aprilTagDetection : aprilDetections) {
                    if (aprilTagDetection.metadata == null) valid = false;
                }
                if (valid) {
                    AprilTag t1 = new AprilTag(aprilDetections.get(0));
                    AprilTag t2 = new AprilTag(aprilDetections.get(1));
                    boolean t1AngleLarger = t1.bearing > t2.bearing;
                    double angleDifference = t1AngleLarger ? t1.bearing - t2.bearing : t2.bearing - t1.bearing;
                    double connectingLength = lawOfCos(t1.hyp, t2.hyp, angleDifference);
                    double t1Angle = t1AngleLarger ? (3 * Math.PI) / 2 - lawOfSin(connectingLength, t2.hyp, angleDifference)
                            : Math.PI / 2 + lawOfSin(connectingLength, t2.hyp, angleDifference);
                    double t2Angle = t1AngleLarger ? Math.PI / 2 - lawOfSin(connectingLength, t1.hyp, angleDifference)
                            : (3 * Math.PI) / 2 + lawOfSin(connectingLength, t1.hyp, angleDifference);
                    VectorF t1Location = t1.detection.metadata.fieldPosition
                            .subtracted(new VectorF(
                                    (float) (t1.hyp * Math.cos(t1Angle)),
                                    (float) (t1.hyp * Math.sin(t1Angle)),
                                    (float) (t1.detection.ftcPose.z)
                            ));
                    VectorF t2Location = t2.detection.metadata.fieldPosition
                            .subtracted(new VectorF(
                                    (float) (t2.hyp * Math.cos(t2Angle)),
                                    (float) (t2.hyp * Math.sin(t2Angle)),
                                    (float) (t2.detection.ftcPose.z)
                            ));
                    packet.fieldOverlay()
                            .setFill("silver")
                            .strokeLine(t1.detection.metadata.fieldPosition.get(0),
                                    t1.detection.metadata.fieldPosition.get(1),
                                    t1Location.get(0),
                                    t1Location.get(1))
                            .strokeLine(t2.detection.metadata.fieldPosition.get(0),
                                    t2.detection.metadata.fieldPosition.get(1),
                                    t2Location.get(0),
                                    t2Location.get(1));
                }
            }

            for (AprilTagDetection aprilDetection : aprilDetections) {

                if (aprilDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    aprilDetectionID = aprilDetection.id;

                    // Now take action based on this tag's ID code, or store info for later action.


                    VectorF distance = new VectorF((float) X_MULTIPLIER * (float) (SWAP_AXIS ? aprilDetection.ftcPose.y : aprilDetection.ftcPose.x), (float) Y_MULTIPLIER * (float) (SWAP_AXIS ? aprilDetection.ftcPose.x : aprilDetection.ftcPose.y), (float) aprilDetection.ftcPose.z);
                    VectorF location = aprilDetection.metadata.fieldPosition
                            .added(distance);

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
                    packet.fieldOverlay().fillCircle(distance.get(0), distance.get(1), 2);
                    packet.fieldOverlay().fillCircle(location.get(0), location.get(1), 2);
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
                    packet.fieldOverlay().fillCircle(rotDistance.get(0), rotDistance.get(1), 2);
                    rotations.add(aprilDetection.ftcPose.bearing);
                }
            }

            telemetry.addData("IMU YAW", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("April Bearings", rotations.toString());
            packet.fieldOverlay().setFill("silver").fillCircle(drive.pose.position.x, drive.pose.position.y, 2);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Measuring", aprilDetectionID);
            telemetry.addLine(aprilDetections.toString());
            telemetry.update();
            drive.updatePoseEstimate();

        }
    }
}
