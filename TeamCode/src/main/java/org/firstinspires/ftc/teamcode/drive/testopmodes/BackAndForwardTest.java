package org.firstinspires.ftc.teamcode.drive.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveController;
import org.firstinspires.ftc.teamcode.drive.pathplanning.MonkeyPathPlanner;
import org.firstinspires.ftc.teamcode.drive.pathplanning.Path;
import org.firstinspires.ftc.teamcode.drive.pathplanning.Point;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

@TeleOp
@Config
public class BackAndForwardTest extends LinearOpMode {
    private final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
    public static double distance = 60; // inches
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();
        boolean forwarding = true;
        final GamepadEx driver1 = new GamepadEx(gamepad1), driver2 = new GamepadEx(gamepad2);
        final SwerveDriveSubsystem drive = new SwerveDriveSubsystem(hardwareMap, telemetry, false, () -> false);
        final MonkeyPathPlanner pather = new MonkeyPathPlanner(telemetry);
        final  DriveController<MonkeyPathPlanner, SwerveDriveSubsystem> auto =
                new DriveController<>(hardwareMap, pather, drive, telemetry);

        Point startingPoint = new Point(new Pose2d(0,0, new Rotation2d(0)), Path.DEFAULT_TOLERANCE);
        Point endingPoint = new Point(new Pose2d(distance,0, new Rotation2d(0)), Path.DEFAULT_TOLERANCE);

        Path forward = new Path().addPoint(endingPoint);
        Path backward = new Path().addPoint(startingPoint);

        auto.setPath(forward);

        boolean aPressing = false;
        boolean yPressing = false;
        auto.setPosition(new Pose2d(0,0,new Rotation2d(0)));
        auto.run();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Driving As percentage: ", drive.isDrivingAsPercentage());
            telemetry.addData("Button B: ", driver1.getButton(GamepadKeys.Button.B));
            telemetry.addData("Is forwarding: ", forwarding);
            telemetry.addData("X Position: ", auto.getPose().getX());
            telemetry.addData("Y Position: ", auto.getPose().getY());
            telemetry.addData("Is at spot: ", auto.isFinished());
            telemetry.addData("Button Y: ", driver1.getButton(GamepadKeys.Button.Y));
            telemetry.update();
            packet.fieldOverlay()
                    .setFill("blue")
                    .fillCircle(auto.getPose().getX(), auto.getPose().getY(), 50);
            if (!driver1.getButton(GamepadKeys.Button.A)) aPressing = false;
            if (!driver1.getButton(GamepadKeys.Button.Y)) yPressing = false;
            // allow updating the controllers
            if (driver1.getButton(GamepadKeys.Button.B)) {
                pather.updateControllers();
            }
            // different behavior if you are controlling with gamepad or doing the pather
            if (drive.isDrivingAsPercentage()) {
                // press A to switch modes
                if (driver1.getButton(GamepadKeys.Button.A) && !aPressing) {
                    aPressing = true;
                    drive.setDriveAsPercentage(false);
                    auto.setPosition(new Pose2d(0,0,new Rotation2d(0)));
                    continue;
                }
                // default moving code, ripped from SwerveDriveTester
                drive.setTargetVelocity(new ChassisSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightY())); // this is concerning if this works, y should not be left and right on gamepad
                drive.periodic();
            } else {
                // press A to switch modes
                if (driver1.getButton(GamepadKeys.Button.A) && !aPressing) {
                    aPressing = true;
                    drive.setDriveAsPercentage(true);
                    continue;
                }
                // Weird auto stuff to switch directions correctly as well as be interruptible
                auto.run();

                if (driver1.getButton(GamepadKeys.Button.Y) && !yPressing) {
                    yPressing = true;
                    if (auto.isFinished()) {
                        if (forwarding) {
                            auto.setPath(backward);
                            forwarding = false;
                        } else {
                            auto.setPath(forward);
                            forwarding = true;
                        }
                    }
                }

            }
        }
    }
}
