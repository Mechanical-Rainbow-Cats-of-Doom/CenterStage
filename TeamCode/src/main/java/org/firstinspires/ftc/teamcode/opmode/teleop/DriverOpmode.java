package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.ControllerDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleDirectionToggleableMotor;
import org.firstinspires.ftc.teamcode.commands.TogglePowerToggleableMotor;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;
import org.firstinspires.ftc.teamcode.tool.DroneLauncher;
import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.tool.Lift;

public class DriverOpmode extends CommandOpMode {
    private GamepadEx driver1, driver2;
    private SwerveDriveSubsystem drive;
    private Lift lift;
    private DroneLauncher droneLauncher;
    private Intake intake;
    private boolean lastLifterPosition;

    @Override
    public void initialize() {
        // initialize hardware
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        drive = new SwerveDriveSubsystem(hardwareMap, telemetry, true,
                () -> driver1.getButton(GamepadKeys.Button.B));
        lift = new Lift(hardwareMap, driver2);
        droneLauncher = new DroneLauncher(hardwareMap, () -> driver2.getButton(GamepadKeys.Button.LEFT_BUMPER));
        intake = new Intake(hardwareMap);
        Servo lifter = hardwareMap.get(Servo.class, "lifter");

        // schedule all commands
        schedule(new ControllerDriveCommand(drive, driver1));
        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ToggleDirectionToggleableMotor(intake));
        driver2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new TogglePowerToggleableMotor(intake));
        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(()->lift.toggleClawOpen());
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(()->lift.toggleAutomatic());
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(()->{
            lifter.setPosition(lastLifterPosition ? 0 : 1);
            lastLifterPosition = !lastLifterPosition;
        });

        // register unregistered subsystems
        register(drive);
        register(lift);
        register(droneLauncher);
        register(intake);
    }
}
