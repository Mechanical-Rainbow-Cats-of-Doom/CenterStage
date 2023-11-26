package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is currently not being used
 */
@Autonomous
@Disabled
public class bezierTest extends LinearOpMode {
    final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);

    @Override
    public void runOpMode() {
        Vector2d P0 = new Vector2d(-30,-8);
        Vector2d P1 = new Vector2d(-5,-13);
        Vector2d P2 = new Vector2d(3,27);
        Vector2d P3 = new Vector2d(29,19);
        Vector2d R = new Vector2d(-10,30);

        Bezier bezier = new Bezier(P0, P1, P2, P3);
        telemetry.addData("t: ", bezier.newtonRaphson(R, 0.6, Math.pow(10,-6), 10000));
        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
