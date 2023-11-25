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

// as of 11/24: 1.1499
@Autonomous
public class MaxVelocityTest extends LinearOpMode {
    /**
     * Movint time is in secons
     */
    static double movingTime = 2.3;
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true, ()->false);
        ElapsedTime runtime = new ElapsedTime();
        double maxVelocity = 0;
        double currentVelocity = 0;
        Localization2EImpl localization = new Localization2EImpl(hardwareMap);
        waitForStart();
        telemetry.addData("Max Velocity: ", maxVelocity);
        runtime.reset();

        drive.setTargetVelocity(new ChassisSpeeds(1,0,0)); // this is concerning if this works, y should not be left and right on gamepad
        while (runtime.seconds() < movingTime && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Max Velocity: ", maxVelocity);
            telemetry.addData("Current Velocity: ", currentVelocity);
            telemetry.addData("Time: ", runtime.seconds());
            drive.periodic();
            localization.updatePosition();
            /*
            The goal is to find the magnitude of the x and y velocity
            and then update the maximum total velocity
             */
            if (localization.getVelocity() == null) {
                telemetry.addLine("Bruh, fin");
                break;
            }
            // magnitude of the two velocities
            currentVelocity = Math.abs(Math.sqrt(Math.pow(localization.getVelocity().vxMetersPerSecond, 2) + Math.pow(localization.getVelocity().vyMetersPerSecond, 2)));
            if(currentVelocity > maxVelocity) maxVelocity = currentVelocity;
            telemetry.update();
        }
        drive.setTargetVelocity(new ChassisSpeeds(0,0,0));
        while (opModeIsActive() && !isStopRequested()) {
            drive.periodic();
            telemetry.update();
        }
    }
}
