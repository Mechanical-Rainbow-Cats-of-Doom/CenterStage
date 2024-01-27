package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.util.DelayStorage;

@Autonomous
public class DelayConfiguration extends LinearOpMode {
    MultipleTelemetry telemetry;
    GamepadEx driver;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                super.telemetry);
        driver = new GamepadEx(gamepad1);
        while (!isStarted() && !isStopRequested() && opModeIsActive()) {
            telemetry.addLine("---Change Delay---");
            telemetry.addData("Current Delay (sec): ", DelayStorage.seconds);
            telemetry.addLine("DPAD_UP increases delay");
            telemetry.addLine("DPAD_DOWN decreases delay");
            telemetry.addLine("X clears delay");
            telemetry.addLine("A exits");
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
                DelayStorage.addSeconds(1);
            });
            driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
                if (DelayStorage.seconds == 0) return;
                DelayStorage.subtractSeconds(1);
            });
            driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
                DelayStorage.setSeconds(0);
            });
        }
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            DelayStorage.waitForDelay();
        }
    }
}
