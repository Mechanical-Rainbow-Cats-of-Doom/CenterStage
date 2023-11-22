package org.firstinspires.ftc.teamcode.drive.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    /**
     * Movint time is in secons
     */
    static int movingTime = 5;
    private ElapsedTime runtime = new ElapsedTime();
    public double maxVelocity = 0;
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true, ()->false);
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        telemetry.addData("Max Velocity: ", maxVelocity);
        runtime.reset();
        // TODO: replace all values of this with the chassis speed from localization after fin implements
        ChassisSpeeds placeHolder = new ChassisSpeeds();
        // TODO: put in code to run the robot forward
        while (runtime.seconds() < movingTime) {
            /*
            The goal is to find the magnitude of the x and y velocity
            and then update the maximum total velocity
             */
            double magnitude = Math.abs(Math.sqrt(Math.pow(placeHolder.vxMetersPerSecond, 2) + Math.pow(placeHolder.vxMetersPerSecond, 2)));
            if(magnitude > maxVelocity) maxVelocity = magnitude;
            telemetry.update();
        }
        // TODO: put in code to stop the robot
    }
}
