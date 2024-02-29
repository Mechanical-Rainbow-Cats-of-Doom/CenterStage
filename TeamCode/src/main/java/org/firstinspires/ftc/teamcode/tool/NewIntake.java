package org.firstinspires.ftc.teamcode.tool;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.SetIntakeCommand;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class NewIntake extends SubsystemBase {
    public static double TRIGGER_DEADZONE = 0.4;
    public static double INTAKE_POWER = 0.75;

    private final Motor intakeMotor;
    private final ServoEx intakeHeight;
    private final GamepadEx toolGamepad;
    private DoubleSupplier state = State.OFF;
    private DoubleSupplier height = DefaultHeight.UP;
    private boolean leftTriggerPreviouslyDepressed = false;
    private boolean rightTriggerPreviouslyDepressed = false;

    public enum State implements DoubleSupplier {
        OFF(() -> 0),
        FORWARD(() -> INTAKE_POWER),
        BACKWARD(() -> -INTAKE_POWER);

        private final DoubleSupplier power;

        State(DoubleSupplier power) {
            this.power = power;
        }

        public double getAsDouble() {
            return power.getAsDouble();
        }
    }

    public enum DefaultHeight implements DoubleSupplier {
        UP(5, 0.24),
        PIXEL_HOVER(6, 0.58),
        PIXEL_5(4, 0.645),
        PIXEL_4(3, 0.664),
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

    public NewIntake(HardwareMap map, GamepadEx toolGamepad, NewLift lift) {
        intakeMotor = new MotorGroup(new Motor(map, "intake"), new Motor(map, "intake2"));

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
        periodic();
    }

    public NewIntake(HardwareMap map) {
        this(map, null, null);
    }

    public void setState(DoubleSupplier state) {
        this.state = state;
    }

    public void setHeight(DoubleSupplier height) {
        this.height = height;
    }

    @Override
    public void periodic() {
        intakeMotor.set(state.getAsDouble());
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
        private final DoubleSupplier speed;
        private final DoubleSupplier intakeHeight;
        private final BooleanSupplier finished;
        private boolean started = false;

        public SpinIntake(BooleanSupplier finished, DoubleSupplier speed, DoubleSupplier intakeHeight) {
            this.speed = speed;
            this.intakeHeight = intakeHeight;
            this.finished = finished;
        }

        public SpinIntake(BooleanSupplier finished, DoubleSupplier intakeHeight) {
            this.speed = State.FORWARD;
            this.intakeHeight = intakeHeight;
            this.finished = finished;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!started) {
                setState(speed);
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

    public Action spinIntakeAction(BooleanSupplier finished, DoubleSupplier speed, DoubleSupplier intakeHeight) {
        return new SpinIntake(finished, speed, intakeHeight);
    }

    public Action spinIntakeAction(PixelSensor pixelSensor, int toCollect, DoubleSupplier intakeHeight) {
        return new SpinIntake(() -> {
            pixelSensor.periodic();
            return toCollect >= pixelSensor.getPixelCount();
        }, intakeHeight);
    }

    public Action spinIntakeAction(PixelSensor pixelSensor, int toCollect, DoubleSupplier speed, DoubleSupplier intakeHeight) {
        return new SpinIntake(() -> {
            pixelSensor.periodic();
            return toCollect >= pixelSensor.getPixelCount();
        }, speed, intakeHeight);
    }

    public Action spinIntakeAction(double timeSeconds, DoubleSupplier speed, DoubleSupplier intakeHeight) {
        AtomicBoolean started = new AtomicBoolean(false);
        Timing.Timer timer = new Timing.Timer((long) (timeSeconds * 1000), TimeUnit.MILLISECONDS);
        return new SpinIntake(() -> {
            if(!started.get()) {
                timer.start();
                started.set(true);
            }
            return timer.done();
        }, speed, intakeHeight);
    }

    public Action spinIntakeAction(double timeSeconds, DoubleSupplier intakeHeight) {
        AtomicBoolean started = new AtomicBoolean(false);
        Timing.Timer timer = new Timing.Timer((long) (timeSeconds * 1000), TimeUnit.MILLISECONDS);
        return new SpinIntake(() -> {
            if(!started.get()) {
                timer.start();
                started.set(true);
            }
            return timer.done();
        }, intakeHeight);
    }

    public Action spinIntakeTimerAction(PixelSensor pixelSensor, int toCollect,
                                        double timeSecondsOverride, DoubleSupplier speed,
                                        DoubleSupplier intakeHeight) {
        AtomicBoolean started = new AtomicBoolean(false);
        Timing.Timer timer = new Timing.Timer((long) (timeSecondsOverride * 1000), TimeUnit.MILLISECONDS);
        return new SpinIntake(() -> {
            if(!started.get()) {
                timer.start();
                started.set(true);
            }
            pixelSensor.periodic();
            return timer.done() || toCollect >= pixelSensor.getPixelCount();
        }, speed, intakeHeight);
    }


    public Action spinIntakeTimerAction(PixelSensor pixelSensor, int toCollect,
                                        double timeSecondsOverride, DoubleSupplier intakeHeight) {
        AtomicBoolean started = new AtomicBoolean(false);
        Timing.Timer timer = new Timing.Timer((long) (timeSecondsOverride * 1000), TimeUnit.MILLISECONDS);
        return new SpinIntake(() -> {
            if(!started.get()) {
                timer.start();
                started.set(true);
            }
            pixelSensor.periodic();
            return timer.done() || toCollect >= pixelSensor.getPixelCount();
        }, intakeHeight);
    }

    public Action setIntakeHeightAction(DoubleSupplier height) {
        return telemetryPacket -> {
            setHeight(height);
            periodic();
            return false;
        };
    }
}
