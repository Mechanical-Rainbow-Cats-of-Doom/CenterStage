package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.ConfigChangeDetector;
import org.firstinspires.ftc.teamcode.tool.NewLift;
import org.firstinspires.ftc.teamcode.tool.old.OldLift;

import java.util.Collections;

@TeleOp
public class NewLiftPositionFromConfig extends LinearOpMode {
    @Config
    public static class LiftPositionConfig {
        public static int liftTicks;
        public static boolean armOut;
        public static double armRoll;
        public static double armLength;
        public static NewLift.LiftPosition.Time liftMoveTime;
        public static NewLift.LiftPosition.Time armRollTime;
        public static NewLift.LiftPosition.Time armYawTime;
        public static NewLift.LiftPosition.Time armLengthTime;
        public static NewLift.LiftPosition.Time clawTime;
        public static boolean clawOpen;
        public static NewLift.LiftPosition.Default defaultPosition = NewLift.LiftPosition.Default.DOWN;
        public static boolean useDefault = false;

        private static final ConfigChangeDetector<LiftPositionConfig> changeDetector =
                new ConfigChangeDetector<>(LiftPositionConfig.class, Collections.singletonList("changeDetector"));


        public static NewLift.LiftPosition getCustomLiftPosition() {
            return useDefault ? defaultPosition : new NewLift.LiftPosition.Custom(liftTicks, armOut,
                    armRoll, armLength, clawOpen, liftMoveTime, armRollTime, armYawTime, armLengthTime, clawTime);
        }

        public static boolean changeDetected() {
            return changeDetector.updateAndHasChanged();
        }
    }

    private MultipleTelemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        GamepadEx driver2 = new GamepadEx(gamepad2);
        NewLift lift = new NewLift(hardwareMap, driver2, false, false);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        long lastPositionTime = System.currentTimeMillis();
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(lift::toggleAutomatic);

        while(!isStopRequested()) {
            CommandScheduler.getInstance().run();
            if(LiftPositionConfig.changeDetected()) {
                lift.setPosition(LiftPositionConfig.getCustomLiftPosition());
                lastPositionTime = System.currentTimeMillis();
            }
            lift.periodic();
            telemetry.addData("Current Run Mode", lift.isAutomatic() ? "automatic" : "manual");
            telemetry.addData("Lift Encoder Position", lift.getCurrentLiftPosition());
            telemetry.addData("Lift Power", lift.getPower());
            if(lift.isAutomatic()) {
                telemetry.addData("Current Lift State", lift.getState().toString());
                telemetry.addData("Lift Position Error", lift.getPower());
            }
            telemetry.addData("Time Since Last Update", System.currentTimeMillis() - lastPositionTime);
            telemetry.update();
        }
    }
}
