package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

public class NewIntake extends SubsystemBase {
    private final MotorEx intakeMotor;
    private final ServoEx intakeHeight;
    private State state = State.OFF;
    private DoubleSupplier height = DefaultHeight.UP;

    public enum State {
        OFF(0),
        FORWARD(1),
        BACKWARD(-1);

        private final double power;

        State(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public enum DefaultHeight implements DoubleSupplier {
        UP(0.24),
        PIXEL_5(0.645),
        PIXEL_4(0.665),
        PIXEL_3(0.685),
        PIXEL_2(0.71),
        BOTTOM(0.73);

        private double height;

        DefaultHeight(double height) {
            this.height = height;
        }

        @Override
        public double getAsDouble() {
            return height;
        }
    }

    public NewIntake(HardwareMap map) {
        intakeMotor = new MotorEx(map, "intake");
        intakeHeight = new SimpleServo(map, "intakeHeight", 0, 1);
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setHeight(DoubleSupplier height) {
        this.height = height;
    }

    @Override
    public void periodic() {
        intakeMotor.set(state.power);
        intakeHeight.setPosition(height.getAsDouble());
    }
}
