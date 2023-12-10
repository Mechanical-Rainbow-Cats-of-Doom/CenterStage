package org.firstinspires.ftc.teamcode.common.hardware;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DebugMotor extends Motor {
    public DebugMotor(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public double getCurrentError() {
        return positionController.calculate(getDistance());
    }
}
