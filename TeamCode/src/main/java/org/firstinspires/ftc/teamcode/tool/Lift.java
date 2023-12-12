package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.DebugMotor;

@Config
public class Lift extends SubsystemBase {
    public interface LiftPosition {
        int getLiftTicks();
        float getServoPosition();
        boolean isClawOpen();
        boolean shouldForceStartOpen();


        enum Default implements LiftPosition {
            DOWN(0, 0, false, true),
            ONE(1700, 0, true, false),
            TWO(2900, 0, true, false),
            THREE(4200, 0, true, false);

            private final int liftTicks;
            private final float servoPosition;
            private final boolean openClaw;
            private final boolean forceStartOpen;

            Default(int liftTicks, float servoPosition, boolean openClaw, boolean forceStartOpen) {
                this.liftTicks = liftTicks;
                this.servoPosition = servoPosition;
                this.openClaw = openClaw;
                this.forceStartOpen = forceStartOpen;
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

            @Override
            public boolean shouldForceStartOpen() {
                return forceStartOpen;
            }
        }

        class Custom implements LiftPosition {
            private final int liftTicks;
            private final float servoPosition;
            private final boolean clawOpen;
            private final boolean forceStartOpen;

            public Custom(int liftTicks, float servoPosition, boolean clawOpen, boolean forceStartOpen) {
                this.liftTicks = liftTicks;
                this.servoPosition = servoPosition;
                this.clawOpen = clawOpen;
                this.forceStartOpen = forceStartOpen;
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

            @Override
            public boolean shouldForceStartOpen() {
                return forceStartOpen;
            }
        }
    }

    public enum State {
        STARTING_MOVE(false, false, false),
        EARLY_RUN_CLAW(false, false, false),
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

    public static double CLAW_OPEN_POSITION = 0.14;
    public static double CLAW_CLOSED_POSITION = 0.12;
    public static double LIFT_POWER = 1;
    public static double kP = 1e-2;
    public static double positionTolerence = 20;
    public static int positionLimit = 4200;

    private final Motor liftMotor;
    private final Servo clawRotationServo, clawServo;
    private final GamepadEx toolGamepad;
    private LiftPosition position = LiftPosition.Default.DOWN;
    private State state = State.AT_POSITION;
    private boolean clawOpen = false;
    private boolean automatic = true;
    private Motor.RunMode runMode = Motor.RunMode.PositionControl;
    private double lastKP = kP;
    private double lastPositionTolerence = positionTolerence;

    public Lift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug) {
        this.toolGamepad = toolGamepad;
        if(debug) {
            liftMotor = new DebugMotor(hardwareMap, "lift1", Motor.GoBILDA.RPM_223);
        } else {
            liftMotor = new Motor(hardwareMap, "lift1", Motor.GoBILDA.RPM_223);
        }
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.setInverted(true);
        liftMotor.resetEncoder();
        liftMotor.setPositionCoefficient(kP);
        liftMotor.setPositionTolerance(positionTolerence);
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

    public Lift(HardwareMap hardwareMap, boolean debug) {
        this(hardwareMap, null, debug);
    }

    public Lift(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public Lift(HardwareMap hardwareMap, GamepadEx toolGamepad) {
        this(hardwareMap, toolGamepad, false);
    }

    @Override
    public void periodic() {
        if(kP != lastKP) {
            liftMotor.setPositionCoefficient(kP);
            lastKP = kP;
        }
        if(positionTolerence != lastPositionTolerence) {
            liftMotor.setPositionTolerance(positionTolerence);
            lastPositionTolerence = positionTolerence;
        }
        if(!automatic) {
            if(runMode != Motor.RunMode.RawPower) {
                liftMotor.setRunMode(Motor.RunMode.RawPower);
                runMode = Motor.RunMode.RawPower;
            }
            liftMotor.set(toolGamepad.getLeftY() * LIFT_POWER);
            return;
        }
        if(runMode != Motor.RunMode.PositionControl) {
            liftMotor.setRunMode(Motor.RunMode.PositionControl);
            runMode = Motor.RunMode.PositionControl;
        }

        if(position == null) {
            if(state == State.STARTING_MOVE) {
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                liftMotor.set(LIFT_POWER);
                state = State.AT_POSITION;
            }
            return;
        }
        if(liftMotor.atTargetPosition()) {
            liftMotor.set(0);
        } else {
            liftMotor.set(LIFT_POWER);
        }
        switch(state) {
            case STARTING_MOVE:
                state = State.EARLY_RUN_CLAW;
            case EARLY_RUN_CLAW:
                if(position.shouldForceStartOpen() && clawServo.getPosition() != CLAW_OPEN_POSITION) {
                    clawServo.setPosition(CLAW_OPEN_POSITION);
                    clawOpen = true;
                }
                state = State.RUNNING_LIFT;
                liftMotor.setTargetPosition(Math.max(0, Math.min(positionLimit, position.getLiftTicks())));
                liftMotor.set(LIFT_POWER);
            case RUNNING_LIFT:
                if(!liftMotor.atTargetPosition()) break;
                state = State.ROTATING_CLAW;
                clawRotationServo.setPosition(position.getServoPosition());
            case ROTATING_CLAW:
                liftMotor.set(0);
                if(clawRotationServo.getPosition() != position.getServoPosition()) break;
                state = State.RUNNING_CLAW;
                clawServo.setPosition(position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION);
            case RUNNING_CLAW:
                if(clawServo.getPosition() != (position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION)) break;
                clawOpen = position.isClawOpen();
                state = State.AT_POSITION;
        }
    }

    public int getCurrentLiftPosition() {
        return liftMotor.getCurrentPosition();
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

    public boolean isAutomatic() {
        return this.automatic;
    }

    public void toggleAutomatic() {
        automatic = !automatic;
    }

    public State getState() {
        return state;
    }

    public double getError() {
        return liftMotor instanceof DebugMotor ? ((DebugMotor) liftMotor).getCurrentError() : 0;
    }

    public double getPower() {
        return liftMotor.get();
    }
}
