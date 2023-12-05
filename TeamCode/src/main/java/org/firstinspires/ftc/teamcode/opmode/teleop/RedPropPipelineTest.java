package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@TeleOp
public class RedPropPipelineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                true, new PropPipeline.PropPipelineDashboardConfig());
        int lastResult = -1;
        while(!isStopRequested()) {
            if(gamepad1.a) {
                float startTime = System.currentTimeMillis() / 1000f;
                lastResult = detector.run(() -> {
                    int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
                    telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
                            (time > 2 ? "." : "") +
                            (time > 3 ? "." : ""));
                    telemetry.update();
                });
                detector.reset();
            }
            if(lastResult >= 0) {
                telemetry.addData("Last Result", lastResult);
                telemetry.addLine();
            }
            telemetry.addLine("Press A to run detector.");
            telemetry.update();
        }
    }
}
