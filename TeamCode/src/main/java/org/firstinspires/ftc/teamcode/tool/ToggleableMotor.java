package org.firstinspires.ftc.teamcode.tool;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ToggleableMotor extends SubsystemBase {
    private final double maxMotorPower;
    private final DcMotorSimple motor;

    private boolean currState, lastState = false;

    public ToggleableMotor(DcMotorSimple motor, double maxMotorPower, boolean startingState) {
        this.motor = motor;
        this.maxMotorPower = maxMotorPower;
        this.currState = startingState;
        periodic();
    }

    public ToggleableMotor(@NonNull HardwareMap hardwareMap, String motorName, double maxMotorPower,
                           boolean startingState) {
        this(hardwareMap.get(DcMotorSimple.class, motorName), maxMotorPower, startingState);
    }

    public ToggleableMotor(DcMotorSimple motor) {
        this(motor, 1, false);
    }

    public ToggleableMotor(HardwareMap hardwareMap, String motorName) {
        this(hardwareMap.get(DcMotorSimple.class, motorName));
    }

    @Override
    public void periodic() {
        if (currState != lastState) updateMotor();
        lastState = currState;
    }

    private void updateMotor() {
        this.motor.setPower(maxMotorPower * (currState ? 1 : -1));
    }

    public boolean toggleState() {
        this.currState = !currState;
        return !currState;
    }

    public void setState(boolean state) {
        this.currState = state;
    }

    public boolean getState() {
        return currState;
    }
}
