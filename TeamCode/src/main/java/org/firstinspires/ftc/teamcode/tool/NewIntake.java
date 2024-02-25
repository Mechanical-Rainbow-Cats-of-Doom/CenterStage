package org.firstinspires.ftc.teamcode.tool;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.SetIntakeCommand;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class NewIntake extends SubsystemBase {
    public static double TRIGGER_DEADZONE = 0.4;

    private final MotorEx intakeMotor;
    private final ServoEx intakeHeight;
    private final GamepadEx toolGamepad;
    private State state = State.OFF;
    private DoubleSupplier height = DefaultHeight.UP;
    private boolean leftTriggerPreviouslyDepressed = false;
    private boolean rightTriggerPreviouslyDepressed = false;

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
        UP(5, 0.24),
        PIXEL_5(4, 0.645),
        PIXEL_4(3, 0.665),
        PIXEL_3(2, 0.685),
        PIXEL_2(1, 0.71),
        BOTTOM(0, 0.73);

        private final int index;
        private final double height;

        DefaultHeight(int index, double height) {
            this.index = index;
            this.height = height;
        }

        @Override
        public double getAsDouble() {
            return height;
        }

        public int getIndex() {
            return index;
        }

        public static DefaultHeight numberToHeight(int number) {
            number = number % 6;
            if(number < 0) {
                number += 6;
            }
            switch(number) {
                case 0:
                    return BOTTOM;
                case 1:
                    return PIXEL_2;
                case 2:
                    return PIXEL_3;
                case 3:
                    return PIXEL_4;
                case 4:
                    return PIXEL_5;
                case 5:
                default:
                    return UP;
            }
        }
    }

    public NewIntake(HardwareMap map, GamepadEx toolGamepad) {
        intakeMotor = new MotorEx(map, "intake");
        intakeHeight = new SimpleServo(map, "intakeHeight", 0, 1);
        intakeMotor.setInverted(true);

        if(toolGamepad != null) {
            toolGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whenReleased(new SetIntakeCommand(this, State.OFF))
                    .whenHeld(new SetIntakeCommand(this, State.FORWARD));

            toolGamepad.getGamepadButton(GamepadKeys.Button.B)
                    .whenReleased(new SetIntakeCommand(this, State.OFF))
                    .whenHeld(new SetIntakeCommand(this, State.BACKWARD));
        }
        this.toolGamepad = toolGamepad;
    }

    public NewIntake(HardwareMap map) {
        this(map, null);
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

        if(toolGamepad != null) {
            boolean leftTriggerDepressed = toolGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_DEADZONE;
            boolean rightTriggerDepressed = toolGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE;

            if(leftTriggerDepressed && rightTriggerDepressed) {
                height = DefaultHeight.UP;
            } else if(leftTriggerDepressed && !leftTriggerPreviouslyDepressed) {
                change(1);
            } else if(rightTriggerDepressed && !rightTriggerPreviouslyDepressed) {
                change(-1);
            }

            leftTriggerPreviouslyDepressed = leftTriggerDepressed;
            rightTriggerPreviouslyDepressed = rightTriggerDepressed;
        }
    }

    private void change(int num) {
        if(!(height instanceof DefaultHeight)) {
            height = DefaultHeight.UP;
            return;
        }

        DefaultHeight defHeight = (DefaultHeight) height;

        height = DefaultHeight.numberToHeight(defHeight.getIndex() + num);
    }

    private class SpinIntake implements Action {
        private final DoubleSupplier intakeHeight;
        private final BooleanSupplier finished;
        private boolean started = false;

        public SpinIntake(BooleanSupplier finished, DoubleSupplier intakeHeight) {
            this.intakeHeight = intakeHeight;
            this.finished = finished;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!started) {
                setState(State.FORWARD);
                setHeight(intakeHeight);
                started = true;
            }
            boolean done = finished.getAsBoolean();
            if(done) {
                setState(State.OFF);
            }
            periodic();
            return !done;
        }
    }

    public Action spinIntakeAction(BooleanSupplier finished, DoubleSupplier intakeHeight) {
        return new SpinIntake(finished, intakeHeight);
    }

    public Action spinIntakeAction(PixelSensor pixelSensor, int toCollect, DoubleSupplier intakeHeight) {
        return new SpinIntake(() -> {
            pixelSensor.periodic();
            return toCollect >= pixelSensor.getPixelCount();
        }, intakeHeight);
    }

    public Action spinIntakeAction(double timeSeconds, DoubleSupplier intakeHeight) {
        boolean started = false;
        Timing.Timer timer = new Timing.Timer((long) (timeSeconds * 1000), TimeUnit.MILLISECONDS);
        return new SpinIntake(() -> {
            if(!started) {
                timer.start();
            }
            return timer.done();
        }, intakeHeight);
    }


}
