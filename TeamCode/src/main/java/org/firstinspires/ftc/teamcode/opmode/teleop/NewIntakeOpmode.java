package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tool.NewIntake;

@Config
@TeleOp
public class NewIntakeOpmode extends LinearOpMode {
    public static NewIntake.State state = NewIntake.State.OFF;
    public static double height = 0;
    public static NewIntake.DefaultHeight setHeight = NewIntake.DefaultHeight.UP;
    public static boolean useSetPosition = true;

    @Override
    public void runOpMode() throws InterruptedException {
        NewIntake intake = new NewIntake(hardwareMap);

        waitForStart();
        while(!isStopRequested()) {
            intake.setState(state);
            if(useSetPosition) {
                intake.setHeight(setHeight);
            } else {
                intake.setHeight(() -> height);
            }

            intake.periodic();
        }
    }
}
