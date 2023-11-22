package org.firstinspires.ftc.teamcode.drive.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;

@TeleOp
public class MaxAngularVelocityTest extends LinearOpMode {
    /**
     * Movint time is in secons
     */
    static int movingTime = 5;
    private ElapsedTime runtime = new ElapsedTime();
    public double maxAngularVelocity = 0;
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        Localization2EImpl localization = new Localization2EImpl(hardwareMap);
        waitForStart();
        telemetry.addData("Max Angular Velocity: ", maxAngularVelocity);
        runtime.reset();
        // TODO: put in code to spin the robot
        while (runtime.seconds() < movingTime) {
            /*
            the goal is to update the max angular velocity to the max value between the
            max angular velocity and the current angular velocity
             */
            if(Math.abs(localization.getVelocity().omegaRadiansPerSecond) > maxAngularVelocity)
                maxAngularVelocity = Math.abs(localization.getVelocity().omegaRadiansPerSecond);
            telemetry.update();

        }
        // TODO: put in code to stop the robot
    }
}
