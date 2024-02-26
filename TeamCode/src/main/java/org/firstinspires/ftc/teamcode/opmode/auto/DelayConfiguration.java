package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.util.DelayStorage;

@Autonomous
@Disabled
public class DelayConfiguration extends LinearOpMode {
    MultipleTelemetry telemetry;
    GamepadEx driver;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                super.telemetry);
        driver = new GamepadEx(gamepad1);
        ToggleButtonReader dpadUp = new ToggleButtonReader(driver, GamepadKeys.Button.DPAD_UP);
        ToggleButtonReader dpadDown = new ToggleButtonReader(driver, GamepadKeys.Button.DPAD_DOWN);

        ToggleButtonReader dpadLeft = new ToggleButtonReader(driver, GamepadKeys.Button.DPAD_LEFT);
        ToggleButtonReader dpadRight = new ToggleButtonReader(driver, GamepadKeys.Button.DPAD_RIGHT);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("---Change Delay---");
            telemetry.addData("Current Delay (sec): ", DelayStorage.seconds);
            telemetry.addLine("DPAD_UP increases delay by 1");
            telemetry.addLine("DPAD_DOWN decreases delay by 1");
            telemetry.addLine("DPAD_RIGHT increases delay by .1");
            telemetry.addLine("DPAD_LEFT decreases delay by .1");
            telemetry.addLine("X clears delay");
            telemetry.addLine("A exits");
            telemetry.update();

            dpadUp.readValue();
            dpadDown.readValue();
            dpadLeft.readValue();
            dpadRight.readValue();

            if(dpadUp.wasJustPressed()) {
                DelayStorage.addSeconds(1);
            }
            if(dpadDown.wasJustPressed()) {
                DelayStorage.subtractSeconds(1);
            }
            if(dpadRight.wasJustPressed()) {
                DelayStorage.addSeconds(.1);
            }
            if(dpadLeft.wasJustPressed()) {
                DelayStorage.subtractSeconds(.1);
            }
        }
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            DelayStorage.waitForDelay();
        }
    }
}
