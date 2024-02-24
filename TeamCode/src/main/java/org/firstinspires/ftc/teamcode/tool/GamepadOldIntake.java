package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

public class GamepadOldIntake extends OldIntake {
    private final ButtonReader toggle;
    public GamepadOldIntake(HardwareMap hardwareMap, BooleanSupplier toggler) {
        super(hardwareMap);
        this.toggle = new ButtonReader(toggler);
    }

    @Override
    public void periodic() {
        toggle.readValue();
        if (toggle.wasJustReleased()) {
            toggleState();
        }
        super.periodic();
    }
}
