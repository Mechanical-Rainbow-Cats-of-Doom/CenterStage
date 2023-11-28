package org.firstinspires.ftc.teamcode.drive.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

//as of 11/27 2.9487
@Autonomous
public class MaxAngularVelocityTest extends LinearOpMode {
    /**
     * Movint time is in secons
     */
    static double movingTime = 5;
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true, ()->false);
        ElapsedTime runtime = new ElapsedTime();

        double maxAngularVelocity = 0;

        Localization2EImpl localization = new Localization2EImpl(hardwareMap);
        telemetry.update();
        waitForStart();
        telemetry.addData("Max Angular Velocity: ", maxAngularVelocity);
        runtime.reset();

        drive.setTargetVelocity(new ChassisSpeeds(0,0,1)); // this is concerning if this works, y should not be left and right on gamepad
        while (runtime.seconds() < movingTime && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Max Angular Velocity: ", maxAngularVelocity);
            telemetry.addData("Time: ", runtime.seconds());
            drive.periodic();
            localization.updatePosition();
            /*
            the goal is to update the max angular velocity to the max value between the
            max angular velocity and the current angular velocity
             */
            if(Math.abs(localization.getVelocity().omegaRadiansPerSecond) > maxAngularVelocity)
                maxAngularVelocity = Math.abs(localization.getVelocity().omegaRadiansPerSecond);
            telemetry.update();

        }
        drive.setTargetVelocity(new ChassisSpeeds(0,0,0)); // this is concerning if this works, y should not be left and right on gamepad
        while (opModeIsActive() && !isStopRequested()) {
            drive.periodic();
            telemetry.update();
        }
    }
}
