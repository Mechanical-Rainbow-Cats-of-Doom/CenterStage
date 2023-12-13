package org.firstinspires.ftc.teamcode.tool;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class ToggleableMotor extends SubsystemBase {
    private double maxMotorPower;
    private final DcMotorSimple motor;

    private boolean currState, lastState = false, on = false, lastOn = false, maxMotorSpeedChanged;

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
        this(motor, 1, true);
    }

    public ToggleableMotor(HardwareMap hardwareMap, String motorName) {
        this(hardwareMap.get(DcMotorSimple.class, motorName));
    }

    @Override
    public void periodic() {
        if (currState != lastState || lastOn != on || maxMotorSpeedChanged) updateMotor();
        lastState = currState;
        lastOn = on;
    }

    private void updateMotor() {
        this.motor.setPower(maxMotorPower * (on ? (currState ? 1 : -1) : 0));
    }

    public boolean toggleState() {
        this.currState = !currState;
        return !currState;
    }

    public boolean toggleOn() {
        on = !on;
        if(on) {
            this.currState = true;
        }
        return !on;
    }

    public void setState(boolean state) {
        this.currState = state;
    }

    public void setMaxMotorPower(double power) {
        this.maxMotorPower = power;
        this.maxMotorSpeedChanged = true;
    }

    public boolean getState() {
        return currState;
    }

    public boolean getOn() {
        return on;
    }
}
