package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

public class GamepadIntake extends Intake {
    private final ButtonReader toggle;
    public GamepadIntake(HardwareMap hardwareMap, BooleanSupplier toggler) {
        super(hardwareMap);
        this.toggle = new ButtonReader(toggler);
    }

    @Override
    public void periodic() {
        toggle.readValue();
        if (toggle.wasJustReleased()) {
            toggle();
        }
        super.periodic();
    }
}
