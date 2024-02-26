package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ControllerDriveCommand;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.tool.DroneLauncher;
import org.firstinspires.ftc.teamcode.tool.NewIntake;
import org.firstinspires.ftc.teamcode.tool.NewLift;

@TeleOp(name = "🐟 DRIVER 🐟")
public class MecanumOpMode extends CommandOpMode {
    private final Telemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    private GamepadEx driver1, driver2;
    private MecanumDriveSubsystem drive;
    private NewLift lift;
    private DroneLauncher droneLauncher;
    private NewIntake intake;
    protected boolean squareInputs = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        // initialize hardware
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        final ToggleButtonReader fieldOrientedReader = new ToggleButtonReader(driver1, GamepadKeys.Button.Y);

        drive = new MecanumDriveSubsystem(hardwareMap, () -> {
            fieldOrientedReader.readValue();
            return fieldOrientedReader.getState();
        }, true, telemetry, squareInputs);
        lift = new NewLift(hardwareMap, driver2);
        droneLauncher = new DroneLauncher(hardwareMap, () -> driver1.getButton(GamepadKeys.Button.BACK));
        intake = new NewIntake(hardwareMap, driver2);

        // schedule all commands
        schedule(new ControllerDriveCommand(drive, driver1, () -> false));

//        schedule(new PrepareSafeCommand(intake, lift));
//        CommandBase toggleForward = new SetIntakeCommand.SetIntakeForward(intake);
//        CommandBase toggleBackward = new SetIntakeCommand.SetIntakeBackward(intake);
//        driver2.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(toggleForward).whenReleased(toggleForward);
//        driver2.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(toggleBackward).whenReleased(toggleBackward);
        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(()->{
            if(!lift.isAutomatic()) {
                lift.setClaw(!lift.isClawOpen());
            }
        });
        driver2.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(()->lift.toggleAutomatic());

        // register unregistered subsystems
        register(drive, lift, intake, droneLauncher);
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Lift State", lift.getState().name());
        telemetry.addData("Lift Position", lift.getCurrentLiftPosition());
        telemetry.update();
    }
}
