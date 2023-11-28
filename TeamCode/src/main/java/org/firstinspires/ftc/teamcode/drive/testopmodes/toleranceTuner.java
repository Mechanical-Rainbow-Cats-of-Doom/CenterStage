package org.firstinspires.ftc.teamcode.drive.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;

/**
 * The goal of this op mode is to find the optimal default tolerance of a point
 * As of 11/27 0.1 inches is a fine tolerance
 */
@TeleOp
public class toleranceTuner extends LinearOpMode {
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        Localization2EImpl localization = new Localization2EImpl(hardwareMap);
        Pose2d startPose = localization.getPosition();
        double distance = 0;
        waitForStart();

        /*
        Constantly displays the robots distance from the starting position
         */
        while (opModeIsActive() && !isStopRequested()) {
            localization.updatePosition();
            telemetry.addData("Distance from start: ", distance);
            distance = Math.sqrt(Math.pow(startPose.getX()-localization.getPosition().getX(), 2) +
                    Math.pow(startPose.getY()-localization.getPosition().getY(), 2));
            telemetry.update();
        }
    }
}
