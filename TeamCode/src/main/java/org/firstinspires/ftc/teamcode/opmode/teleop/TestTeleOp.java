package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.swerve.DefaultSwerveDriveCommand;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

// If you are confused how this works, please see TestAuto for more comments
// Also i'm confused too cuz this shit is broke, see SwerveDriveTester for a currently working impl

@TeleOp
public class TestTeleOp extends CommandOpMode {
    private SwerveDriveSubsystem swerveDrive;
    private GamepadEx driver1, driver2;
   // private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);

    @Override
    public void initialize() {
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        swerveDrive = new SwerveDriveSubsystem(hardwareMap, telemetry, true, () -> driver1.getButton(GamepadKeys.Button.B));
        swerveDrive.setDefaultCommand(new PerpetualCommand(
                new DefaultSwerveDriveCommand(
                        swerveDrive,
                        () -> {
                            telemetry.addData("gamepad1 y", driver1.getLeftY());
                            telemetry.addData("gamepad1 x", driver1.getLeftX());
                            telemetry.addData("gamepad1 rightx", driver1.getRightX());
                            return new ChassisSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX());
                        }, () -> false
                )
        ));
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
    
}
