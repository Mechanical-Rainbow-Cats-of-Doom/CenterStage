package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
public class LocalizationTestWithoutMotors extends OpMode {
    public Localization2EImpl localization;
    public GamepadEx driver1;
    public MultipleTelemetry telemetry;
    public double lastRunFrequency = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        localization = new Localization2EImpl(hardwareMap);
        driver1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        localization.updatePosition();

        Pose2d pose = localization.getPosition();
        ChassisSpeeds speeds = localization.getVelocity();
        if(driver1.wasJustPressed(GamepadKeys.Button.X)) {
            localization.setPosition(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));
        }

        if(localization.currentRunCountsTime() >= 500) {
            lastRunFrequency = localization.getRunFrequency();
        }

        telemetry.addLine("Press X to reset position and rotation");
        telemetry.addLine("-----POSITION-----");
        telemetry.addLine("Measured in inches and radians");
        telemetry.addData("X Position", pose.getX());
        telemetry.addData("Y Position", pose.getY());
        telemetry.addData("Rotation", pose.getRotation().getDegrees() + "°");
        telemetry.addLine("------TICKS------");
        telemetry.addData("X Ticks", localization.getXEncoderTicks());
        telemetry.addData("Y Ticks", localization.getYEncoderTicks());
        telemetry.addLine("-----VELOCITY-----");
        telemetry.addLine("Measured in m/s and rads/s");
        telemetry.addData("X Velocity", speeds.vxMetersPerSecond);
        telemetry.addData("Y Velocity", speeds.vyMetersPerSecond);
        telemetry.addData("Rotation Velocity: ", speeds.omegaRadiansPerSecond);
        telemetry.addLine("---MISCELLANEOUS---");
        telemetry.addData("Run Frequency", lastRunFrequency);
        telemetry.update();
    }

    public Pose2d convertToMeters(Pose2d pose) {
        return new Pose2d(pose.getX() / 39.3701, pose.getY() / 39.3701, pose.getRotation());
    }

    public ChassisSpeeds convertToInchesPerSecond(ChassisSpeeds speeds) {
        return new ChassisSpeeds(speeds.vxMetersPerSecond * 39.3701,
                speeds.vxMetersPerSecond * 39.3701, speeds.omegaRadiansPerSecond);
    }
}