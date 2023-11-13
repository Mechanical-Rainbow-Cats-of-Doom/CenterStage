package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

public class GamepadIntake extends Intake {
    private final ToggleButtonReader toggle;
    public GamepadIntake(HardwareMap hardwareMap, BooleanSupplier toggler) {
        super(hardwareMap);
        this.toggle = new ToggleButtonReader(toggler);
    }

    @Override
    public void periodic() {
        toggle.readValue();
        this.setState(toggle.getState());
        super.periodic();
    }
}
