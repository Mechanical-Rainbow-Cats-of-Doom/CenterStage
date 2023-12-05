package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    public Localization2EImpl localization;
    public SwerveDriveSubsystem drive;
    public GamepadEx driver1;
    public MultipleTelemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        localization = new Localization2EImpl(hardwareMap);
        driver1 = new GamepadEx(gamepad1);
        drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true, () -> driver1.getButton(GamepadKeys.Button.B));
        waitForStart();
        localization.initialize();

        while (!isStopRequested()) {
            drive.setTargetVelocity(new ChassisSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX()));
            drive.periodic();
            localization.updatePosition();
            driver1.readButtons();

            Pose2d pose = localization.getPosition();
            ChassisSpeeds speeds = localization.getVelocity();
            if(driver1.wasJustPressed(GamepadKeys.Button.X)) {
                localization.setPosition(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));
            }
            Translation2d robotCoordinates = localization.getRobotCoordinates();

            telemetry.addLine("Press X to reset position and rotation");
            telemetry.addLine("-----POSITION-----");
            telemetry.addLine("Measured in inches and degrees");
            telemetry.addData("X Position", pose.getX());
            telemetry.addData("Y Position", pose.getY());
            telemetry.addData("Rotation", pose.getRotation().getDegrees());
            telemetry.addLine("--RELATIVE MOVEMENT--");
            telemetry.addData("Relative X", robotCoordinates.getX());
            telemetry.addData("Relative Y", robotCoordinates.getY());
            telemetry.addLine("------TICKS------");
            telemetry.addData("X Ticks", localization.getXEncoderTicks());
            telemetry.addData("Y Ticks", localization.getYEncoderTicks());
            telemetry.addLine("-----VELOCITY-----");
            telemetry.addLine("Measured in m/s and rads/s");
            telemetry.addData("X Velocity", speeds.vxMetersPerSecond);
            telemetry.addData("Y Velocity", speeds.vyMetersPerSecond);
            telemetry.addData("Rotation Velocity", speeds.omegaRadiansPerSecond);
            telemetry.update();
        }
    }

    public Pose2d convertToMeters(Pose2d pose) {
        return new Pose2d(pose.getX() / 39.3701, pose.getY() / 39.3701, pose.getRotation());
    }

    public ChassisSpeeds convertToInchesPerSecond(ChassisSpeeds speeds) {
        return new ChassisSpeeds(speeds.vxMetersPerSecond * 39.3701,
                speeds.vxMetersPerSecond * 39.3701, speeds.omegaRadiansPerSecond);
    }
}
