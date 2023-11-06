package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.localization.AprilTagLocalization;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp
public class AprilTagLocalizationTester extends LinearOpMode {
    MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
        super.telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
        Pose2d[] features = new Pose2d[587];

        AprilTagLocalization localization = new AprilTagLocalization(aprilTagProcessor,
                new Pose2d(), telemetry);
        FtcDashboard.getInstance().startCameraStream(OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1")),
                visionPortal.getFps());

        int n = 0;
        while(!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            if(n == 0) telemetry.addLine("Waiting for camera.");
            if(n == 1) telemetry.addLine("Waiting for camera..");
            if(n == 2) telemetry.addLine("Waiting for camera...");
            n++;
            if(n > 2) n = 0;
            telemetry.update();
        }
        while(!isStopRequested()) {
            telemetry.addData("Time", System.currentTimeMillis());
            localization.updatePosition();

            telemetry.update();
        }
        FtcDashboard.getInstance().stopCameraStream();
    }
}
