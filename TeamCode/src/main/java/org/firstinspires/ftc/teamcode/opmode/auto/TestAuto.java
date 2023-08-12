package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestAuto extends CommandOpMode {

    @Override
    public void initialize() {
        // initialize hardware
        // schedule commands
        // register any subsystems that are unregistered for some reason
        // example of how this is used here https://github.com/FTCLib/FTCLib/blob/v2.1.1/examples/src/main/java/com/example/ftclibexamples/PurePursuitSample.java
    }

    // nothing in here is actually overriding yet I am just putting this here to show off what happens when you run op mode
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}
