package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.SwerveModule;

@TeleOp
public class SwerveEncoderTester extends LinearOpMode {
    final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        final SwerveModule frontL = new SwerveModule(hardwareMap, "frontLeftMotor", "frontLeftServo", "frontLeftEncoder", SwerveModule.Wheel.FRONT_LEFT);
        final SwerveModule frontR = new SwerveModule(hardwareMap, "frontRightMotor", "frontRightServo", "frontRightEncoder", SwerveModule.Wheel.FRONT_RIGHT);
        final SwerveModule backL = new SwerveModule(hardwareMap, "backLeftMotor", "backLeftServo", "backLeftEncoder", SwerveModule.Wheel.BACK_LEFT);
        final SwerveModule backR = new SwerveModule(hardwareMap, "backRightMotor", "backRightServo", "backRightEncoder", SwerveModule.Wheel.BACK_RIGHT);
        final SwerveModule[] swerveModules = new SwerveModule[] {frontL, frontR, backL, backR};
        waitForStart();
        while (opModeIsActive()) {
        for (int i = 0, swerveModulesLength = swerveModules.length; i < swerveModulesLength; i++) {
            SwerveModule module = swerveModules[i];
            module.read();
            module.runTelemetry(Integer.toString(i), telemetry);
        }
        telemetry.update();
    }
    }
}
