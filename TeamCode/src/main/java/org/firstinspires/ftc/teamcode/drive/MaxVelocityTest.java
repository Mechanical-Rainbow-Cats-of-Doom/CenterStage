package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    /**
     * Movint time is in secons
     */
    static int movingTime = 5;
    private ElapsedTime runtime = new ElapsedTime();
    public double maxVelocity = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        while (runtime.seconds() < movingTime) {

        }
    }
}
