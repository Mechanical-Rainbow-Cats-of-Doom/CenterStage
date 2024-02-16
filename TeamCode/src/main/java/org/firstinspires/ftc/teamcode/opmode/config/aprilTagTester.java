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
    IMU imu;
    AprilTagProcessor.Builder aprilBuilder;
    AprilTagProcessor aprilProcessor;
    VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;
    MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    FtcDashboard dashboard = FtcDashboard.getInstance();

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

            // Get a list of AprilTag detections.
            aprilDetections = aprilProcessor.getDetections();

            // Cycle through through the list and process each AprilTag.
            for (AprilTagDetection aprilDetection : aprilDetections) {

                if (aprilDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    aprilDetectionID = aprilDetection.id;

                    // Now take action based on this tag's ID code, or store info for later action.
                    VectorF distance = new VectorF((float) -aprilDetection.ftcPose.y, (float) aprilDetection.ftcPose.x, (float) aprilDetection.ftcPose.z);
                    VectorF location = aprilDetection.metadata.fieldPosition
                            .added(distance);


                    switch (fill) {
                        case 0:
                            packet.fieldOverlay().setFill("red");
                            fill = 1;
                            break;
                        case 1:
                            packet.fieldOverlay().setFill("orange");
                            fill = 2;
                            break;
                        case 2:
                            packet.fieldOverlay().setFill("blue");
                            fill = 0;
                            break;
                    }
                    packet.fieldOverlay().fillCircle(aprilDetection.metadata.fieldPosition.get(0),aprilDetection.metadata.fieldPosition.get(1), 2);
                    packet.fieldOverlay().fillCircle(distance.get(0), distance.get(1), 2);
                    packet.fieldOverlay().fillCircle(location.get(0), location.get(1), 2);
                    rotations.add(aprilDetection.ftcPose.yaw);
                }
            }

            telemetry.addData("IMU YAW", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("April YAWs", rotations.toString());
            packet.fieldOverlay().setFill("yellow").fillCircle(drive.pose.position.x, drive.pose.position.y, 2);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Measuring", aprilDetectionID);
            telemetry.addLine(aprilDetections.toString());
            telemetry.update();
            drive.updatePoseEstimate();

        }
    }
}
