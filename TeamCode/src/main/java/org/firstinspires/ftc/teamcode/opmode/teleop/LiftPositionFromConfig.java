//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.common.util.ConfigChangeDetector;
//import org.firstinspires.ftc.teamcode.tool.old.OldLift;
//
//import java.util.Collections;
//
//@TeleOp
//public class LiftPositionFromConfig extends LinearOpMode {
//    @Config
//    public static class LiftPositionConfig {
//        public static int liftTicks;
//        public static double servoPosition;
//        public static boolean rotateClawEarly;
//        public static boolean clawOpen = true;
//        public static boolean forceStartOpen;
//        public static Lift.LiftPosition.Default defaultPosition = Lift.LiftPosition.Default.DOWN;
//        public static boolean useDefault = false;
//
//        private static final ConfigChangeDetector<LiftPositionConfig> changeDetector =
//                new ConfigChangeDetector<>(LiftPositionConfig.class, Collections.singletonList("changeDetector"));
//
//
//        public static OldLift.LiftPosition getCustomLiftPosition() {
//            return useDefault ? defaultPosition : new OldLift.LiftPosition.Custom(liftTicks, (float)servoPosition, rotateClawEarly, clawOpen, forceStartOpen);
//        }
//
//        public static boolean changeDetected() {
//            return changeDetector.updateAndHasChanged();
//        }
//    }
//
//    private MultipleTelemetry telemetry;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        CommandScheduler.getInstance().reset();
//        GamepadEx driver2 = new GamepadEx(gamepad2);
//        OldLift lift = new OldLift(hardwareMap, driver2, true, false, telemetry);
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
//        waitForStart();
//        long lastPositionTime = System.currentTimeMillis();
//        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(lift::toggleAutomatic);
//
//        while(!isStopRequested()) {
//            CommandScheduler.getInstance().run();
//            if(LiftPositionConfig.changeDetected()) {
//                lift.setPosition(LiftPositionConfig.getCustomLiftPosition());
//                lastPositionTime = System.currentTimeMillis();
//            }
//            lift.periodic();
//            telemetry.addData("Current Run Mode", lift.isAutomatic() ? "automatic" : "manual");
//            telemetry.addData("Lift Encoder Position", lift.getCurrentLiftPosition());
//            telemetry.addData("Lift Power", lift.getPower());
//            if(lift.isAutomatic()) {
//                telemetry.addData("Current Lift State", lift.getState().toString());
//                telemetry.addData("Lift Position Error", lift.getError());
//            }
//            telemetry.addData("Claw Open", lift.getClawOpen());
//            telemetry.addData("Time Since Last Update", System.currentTimeMillis() - lastPositionTime);
//            telemetry.update();
//        }
//    }
//}
