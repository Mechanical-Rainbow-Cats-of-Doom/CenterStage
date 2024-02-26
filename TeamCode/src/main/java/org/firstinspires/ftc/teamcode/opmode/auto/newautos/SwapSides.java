package org.firstinspires.ftc.teamcode.opmode.auto.newautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class SwapSides extends LinearOpMode {
    public static boolean isRed = true;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("set for red:", isRed);
        telemetry.update();

        waitForStart();

        isRed = !isRed;
    }
}
