package org.firstinspires.ftc.teamcode.tool;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoToggle {
    private final double off, on;
    private final Servo servo;
    private boolean currState, lastState = false;

    public ServoToggle(@NonNull HardwareMap hardwareMap, String servoName, double offPos,
                       double onPos, boolean startingState) {
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.off = offPos;
        this.on = onPos;
        updateServo();
    }

    public ServoToggle(@NonNull HardwareMap hardwareMap, String servoName, double offPos,
                       double onPos) {
        this(hardwareMap, servoName, offPos, onPos, false);
    }

    public void update() {
        if (currState != lastState) updateServo();
        lastState = currState;
    }

    private void updateServo() {
        servo.setPosition(currState ? on : off);
    }

    public void setState(boolean state) {
        this.currState = state;
    }

    public boolean toggleState() {
        this.currState = !currState;
        return !currState;
    }

    public boolean getState(boolean state) {
        return currState;
    }
}
