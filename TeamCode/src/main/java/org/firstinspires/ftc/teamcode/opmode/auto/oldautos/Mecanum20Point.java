package org.firstinspires.ftc.teamcode.opmode.auto.oldautos;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.util.DcMotorSimpleGroup;
import org.firstinspires.ftc.teamcode.drive.mecanum.AutoMecanum;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

@Disabled
@Autonomous
@Config
public class Mecanum20Point extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        final ElapsedTime timer = new ElapsedTime();
        final DcMotorSimpleGroup intake = new DcMotorSimpleGroup(
                new Pair<>(hardwareMap.get(DcMotor.class, "intake"), false),
                new Pair<>(hardwareMap.get(CRServo.class, "intakeServo"), true)
        );

        final AutoMecanum mecanum = new AutoMecanum(hardwareMap);

        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                true, PropPipeline.PropPipelineRectsProvider.Default.RED_AUDIENCE_SIDE
        );
        int result = -1;
        float startTime = System.currentTimeMillis() / 1000f;

        waitForStart();
        mecanum.initialize();

        result = detector.run(() -> {
            int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
                    (time > 2 ? "." : "") +
                    (time > 3 ? "." : ""));
            telemetry.update();
        });
        detector.reset();

        switch (result) {
            case -1: // shouldn't happen
                return;
            case 0: // left
                mecanum.goForward(25, 0.5);
                mecanum.rotate(-15, 1, -0.3);
                intake.setPower(-1);
                mecanum.goForward(-2, 0.1, -0.2);
                intake.setPower(0);
            case 1: // middle
                mecanum.goForward(30, 0.5);
                intake.setPower(-1);
                mecanum.goForward(-2, 0.1, -0.2);
                intake.setPower(0);
            case 2: // right
                mecanum.goForward(25, 0.5);
                mecanum.rotate(15, 1, 0.3);
                intake.setPower(-1);
                mecanum.goForward(-2, 0.1, -0.2);
                intake.setPower(0);

        }
    }
}
