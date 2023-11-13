package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private final ToggleableMotor motor;

    public Intake(HardwareMap hardwareMap) {
        this.motor = new ToggleableMotor(hardwareMap, "intake");
    }

    public boolean getState() {
        return motor.getState();
    }

    public boolean toggle() {
        return motor.toggleState();
    }

    public void setState(boolean state) {
        motor.setState(state);
    }

    public void periodic() {
        motor.update();
    }
}
