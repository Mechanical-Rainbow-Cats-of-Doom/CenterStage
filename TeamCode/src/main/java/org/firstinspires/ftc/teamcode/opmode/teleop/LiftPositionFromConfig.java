package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.ConfigChangeDetector;
import org.firstinspires.ftc.teamcode.tool.Lift;

import java.util.Collections;

@TeleOp
public class LiftPositionFromConfig extends LinearOpMode {
    @Config
    public static class LiftPositionConfig {
        public static int liftTicks;
        public static float servoPosition;
        public static boolean clawOpen;
        private static final ConfigChangeDetector<LiftPositionConfig> changeDetector =
                new ConfigChangeDetector<>(LiftPositionConfig.class, Collections.singletonList("changeDetector"));


        public static Lift.LiftPosition.Custom getCustomLiftPosition() {
            return new Lift.LiftPosition.Custom(liftTicks, servoPosition, clawOpen);
        }

        public static boolean changeDetected() {
            return changeDetector.updateAndHasChanged();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        waitForStart();

        while(!isStopRequested()) {
            if(LiftPositionConfig.changeDetected())
                lift.setPosition(LiftPositionConfig.getCustomLiftPosition());
            lift.periodic();
        }
    }
}
