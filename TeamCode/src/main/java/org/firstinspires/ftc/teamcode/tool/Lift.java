package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Lift extends SubsystemBase {
    public interface LiftPosition {
        int getLiftTicks();
        float getServoPosition();
        boolean isClawOpen();


        enum Default implements LiftPosition {
            DOWN(0, 0, false),
            ONE(0, 0, true),
            TWO(0, 0, true),
            THREE(0, 0, true);

            private final int liftTicks;
            private final float servoPosition;
            private final boolean openClaw;

            Default(int liftTicks, float servoPosition, boolean openClaw) {
                this.liftTicks = liftTicks;
                this.servoPosition = servoPosition;
                this.openClaw = openClaw;
            }

            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public float getServoPosition() {
                return servoPosition;
            }

            @Override
            public boolean isClawOpen() {
                return openClaw;
            }
        }

        class Custom implements LiftPosition {
            private final int liftTicks;
            private final float servoPosition;
            private final boolean clawOpen;

            public Custom(int liftTicks, float servoPosition, boolean clawOpen) {
                this.liftTicks = liftTicks;
                this.servoPosition = servoPosition;
                this.clawOpen = clawOpen;
            }

            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public float getServoPosition() {
                return servoPosition;
            }

            @Override
            public boolean isClawOpen() {
                return clawOpen;
            }
        }
    }

    public enum State {
        STARTING_MOVE(false, false, false),
        RUNNING_LIFT(false, false, false),
        ROTATING_CLAW(true, false, false),
        RUNNING_CLAW(true, true, false),
        AT_POSITION(true, true, true);

        public final boolean liftMoved;
        public final boolean servoMoved;
        public final boolean finished;

        State(boolean liftMoved, boolean servoMoved, boolean finished) {
            this.liftMoved = liftMoved;
            this.servoMoved = servoMoved;
            this.finished = finished;
        }

        public boolean isLiftMoved() {
            return liftMoved;
        }

        public boolean isServoMoved() {
            return servoMoved;
        }

        public boolean isFinished() {
            return finished;
        }
    }

    public static float CLAW_OPEN_POSITION = 0;
    public static float CLAW_CLOSED_POSITION = 0;

    private final Motor liftMotor;
    private final Servo clawRotationServo, clawServo;
    private final GamepadEx toolGamepad;
    private LiftPosition position = LiftPosition.Default.DOWN;
    private State state = State.AT_POSITION;
    private boolean clawOpen = false;
    private boolean automatic = true;
    private Motor.RunMode runMode = Motor.RunMode.PositionControl;

    public Lift(HardwareMap hardwareMap, GamepadEx toolGamepad) {
        this.toolGamepad = toolGamepad;
        liftMotor = new MotorGroup(new Motor(hardwareMap, "lift1", Motor.GoBILDA.RPM_435));
        clawRotationServo = hardwareMap.get(Servo.class, "clawRotation");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        liftMotor.setRunMode(Motor.RunMode.PositionControl);
        if (toolGamepad != null) {
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(() -> setPosition(LiftPosition.Default.DOWN));
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                    .whenPressed(() -> setPosition(LiftPosition.Default.ONE));
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                    .whenPressed(() -> setPosition(LiftPosition.Default.TWO));
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(() -> setPosition(LiftPosition.Default.THREE));
        }
    }

    public Lift(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    @Override
    public void periodic() {
        if(!automatic) {
            if(runMode != Motor.RunMode.RawPower) {
                liftMotor.setRunMode(Motor.RunMode.RawPower);
                runMode = Motor.RunMode.RawPower;
            }
            liftMotor.set(toolGamepad.getLeftY());
            return;
        }
        if(runMode != Motor.RunMode.PositionControl) {
            liftMotor.setRunMode(Motor.RunMode.PositionControl);
            runMode = Motor.RunMode.PositionControl;
        }

        if(position == null) {
            if(state == State.STARTING_MOVE) {
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                liftMotor.set(1);
                state = State.AT_POSITION;
            }
            return;
        }
        switch(state) {
            case STARTING_MOVE:
                liftMotor.setTargetPosition(position.getLiftTicks());
                liftMotor.set(1);
                state = State.RUNNING_LIFT;
                break;
            case RUNNING_LIFT:
                if(!liftMotor.atTargetPosition()) break;
                state = State.ROTATING_CLAW;
                clawRotationServo.setPosition(position.getServoPosition());
                break;
            case ROTATING_CLAW:
                if(clawRotationServo.getPosition() != position.getServoPosition()) break;
                if(position.isClawOpen() == clawOpen) {
                    state = State.AT_POSITION;
                    break;
                }
                state = State.RUNNING_CLAW;
                clawServo.setPosition(position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION);
                break;
            case RUNNING_CLAW:
                if(clawServo.getPosition() != (position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION)) break;
                clawOpen = position.isClawOpen();
                state = State.AT_POSITION;
            case AT_POSITION:
                // TODO make sure this keeps the lift in the right position
                break;
        }
    }

    public void toggleClawOpen() {
        if(state != State.AT_POSITION) setPosition(null);
        clawOpen = !clawOpen;
        clawServo.setPosition(clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION);
    }

    public void setPosition(LiftPosition position) {
        this.position = position;
        this.state = State.STARTING_MOVE;
    }

    public void setAutomatic(boolean automatic) {
        this.automatic = automatic;
    }

    public void toggleAutomatic() {
        automatic = !automatic;
    }

    public State getState() {
        return state;
    }
}
