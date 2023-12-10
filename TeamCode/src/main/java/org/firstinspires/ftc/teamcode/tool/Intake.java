package org.firstinspires.ftc.teamcode.tool;

import com.qualcomm.robotcore.hardware.HardwareMap;

// TODO add servo spinning the opposite direction
public class Intake extends ToggleableMotor {
    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap, "intake");
    }
}
