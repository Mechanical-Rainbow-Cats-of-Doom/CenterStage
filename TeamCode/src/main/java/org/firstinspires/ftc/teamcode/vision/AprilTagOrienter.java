package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class AprilTagOrienter extends LinearOpMode {
    IMU imu;
    AprilTagProcessor.Builder aprilBuilder;
    AprilTagProcessor aprilProcessor;
    VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;
    MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    FtcDashboard dashboard = FtcDashboard.getInstance();

    MotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        this.frontLeft = new MotorEx(hardwareMap, "frontLeftMotor");
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setInverted(true);
        this.frontRight = new MotorEx(hardwareMap, "frontRightMotor");
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setInverted(true);
        this.backLeft = new MotorEx(hardwareMap, "backLeftMotor");
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.backRight = new MotorEx(hardwareMap, "backRightMotor");
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();


            // Get a list of AprilTag detections.
            aprilDetections = aprilProcessor.getDetections();
            double averageBearing = 0;

            // Cycle through through the list and process each AprilTag.
            for (AprilTagDetection aprilDetection : aprilDetections) {

                if (aprilDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    averageBearing = aprilDetection.ftcPose.bearing;
                }

            }
            averageBearing = averageBearing/aprilDetections.size();

            double turnGain = 0.01;
            double maxAutoTurn = 0.25;
            double yaw = Range.clip(averageBearing*turnGain, -maxAutoTurn, maxAutoTurn);

            rotate(yaw);

            telemetry.addLine(aprilDetections.toString());
            telemetry.update();
        }
    }

    public void rotate(double yaw) {
        frontLeft.set(-yaw);
        backLeft.set(-yaw);
        frontRight.set(yaw);
        backRight.set(yaw);
    }
}
