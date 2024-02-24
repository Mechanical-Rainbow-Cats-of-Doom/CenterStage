package org.firstinspires.ftc.teamcode.tool.old;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.LiftGoToPositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.DebugMotor;

@Config
public class OldLift extends SubsystemBase {
    public interface LiftPosition {
        int getLiftTicks();
        double getClawRotation();
        boolean rotateClawEarly();
        boolean isClawOpen();
        boolean shouldForceStartOpen();
        LiftPosition getNextLiftPosition();


        enum Default implements LiftPosition {
            DOWN(100, 0.115f, true, false, true),
            DOWN_DONT_DROP(100, 0.115f, true, false, false),
            SAFE(953, 0.115f, true, true, true),
            LOW(2500, 0.45f, false, true, false),
            ONE(3335, 0.45f, false, true, false),
            TWO(5718, 0.45f, false, true, false),
            THREE(8196, 0.45f, false, true, false);

            private final int liftTicks;
            private final double servoPosition;
            private final boolean rotateClawEarly;
            private final boolean openClaw;
            private final boolean forceStartOpen;
            private final LiftPosition next;

            Default(int liftTicks, float servoPosition, boolean rotateClawEarly, boolean openClaw,
                    boolean forceStartOpen, LiftPosition next) {
                this.liftTicks = liftTicks;
                this.servoPosition = servoPosition;
                this.rotateClawEarly = rotateClawEarly;
                this.openClaw = openClaw;
                this.forceStartOpen = forceStartOpen;
                this.next = next;
            }

            Default(int liftTicks, float servoPosition, boolean rotateClawEarly, boolean openClaw,
                    boolean forceStartOpen) {
                this(liftTicks, servoPosition, rotateClawEarly, openClaw, forceStartOpen, null);
            }

            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public double getClawRotation() {
                return servoPosition;
            }

            @Override
            public boolean rotateClawEarly() {
                return rotateClawEarly;
            }

            @Override
            public boolean isClawOpen() {
                return openClaw;
            }

            @Override
            public boolean shouldForceStartOpen() {
                return forceStartOpen;
            }

            @Override
            public LiftPosition getNextLiftPosition() {
                return next;
            }
        }

        class Custom implements LiftPosition {
            private final int liftTicks;
            private final double servoPosition;
            private final boolean rotateClawEarly;
            private final boolean clawOpen;
            private final boolean forceStartOpen;
            private final LiftPosition next;

            public Custom(int liftTicks, float servoPosition, boolean rotateClawEarly,
                          boolean clawOpen, boolean forceStartOpen, LiftPosition next) {
                this.liftTicks = liftTicks;
                this.servoPosition = servoPosition;
                this.clawOpen = clawOpen;
                this.rotateClawEarly = rotateClawEarly;
                this.forceStartOpen = forceStartOpen;
                this.next = next;
            }

            public Custom(int liftTicks, float servoPosition, boolean rotateClawEarly,
                          boolean clawOpen, boolean forceStartOpen) {
                this(liftTicks, servoPosition, rotateClawEarly, clawOpen, forceStartOpen, null);
            }

            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public double getClawRotation() {
                return servoPosition;
            }

            @Override
            public boolean rotateClawEarly() {
                return rotateClawEarly;
            }

            @Override
            public boolean isClawOpen() {
                return clawOpen;
            }

            @Override
            public boolean shouldForceStartOpen() {
                return forceStartOpen;
            }

            @Override
            public LiftPosition getNextLiftPosition() {
                return next;
            }
        }
    }

    public enum State {
        STARTING_MOVE(false, false, false),
        EARLY_RUN_CLAW(false, false, false),
        EARLY_ROTATE_CLAW(false, false, false),
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

    public static double CLAW_OPEN_POSITION = 0.138;
    public static double CLAW_CLOSED_POSITION = 0.12;
    public static double LIFT_POWER = 1;
    public static double kP = 1.2e-2;
    public static double positionTolerence = 20;
    public static int MIN_LIFT_POSITION = 40;
    public static int MAX_LIFT_POSITION = 8196;
    public static double clawRotateRunTimeP = 1000;
    public static double clawOpenTime = 400;
    public static double clawManualSpeed = 100;
    public static double MAX_CLAW_POSITION = 0.115;
    private final Motor liftMotor;
    private final Servo clawRotationServo, clawServo;
    private final GamepadEx toolGamepad;
    private LiftPosition position = LiftPosition.Default.DOWN;
    private State state = State.STARTING_MOVE;
    private boolean clawOpen = true;
    private boolean automatic = true;
    private Motor.RunMode runMode = Motor.RunMode.PositionControl;
    private double lastKP = kP;
    private double lastPositionTolerence = positionTolerence;

    private final ElapsedTime timer = new ElapsedTime();
    private double calculatedRotationTime;
    private final Telemetry telemetry;

    private double lastClawRotation = -1;

    private final boolean isTeleOp;

    public OldLift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug, boolean teleOp,
                Telemetry telemetry) {
        final Motor.GoBILDA motorType = Motor.GoBILDA.RPM_117;
        final String id = "lift1";
        this.toolGamepad = toolGamepad;
        if(debug) {
            liftMotor = new DebugMotor(hardwareMap, id, motorType);
        } else {
            liftMotor = new Motor(hardwareMap, id, motorType);
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
//            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                    .whenPressed(new LiftGoToPositionCommand(this, LiftPosition.Default.DOWN));
//            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                    .whenPressed(new LiftGoToPositionCommand(this, LiftPosition.Default.ONE));
//            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                    .whenPressed(new LiftGoToPositionCommand(this, LiftPosition.Default.TWO));
//            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                    .whenPressed(new LiftGoToPositionCommand(this, LiftPosition.Default.THREE));
//            toolGamepad.getGamepadButton(GamepadKeys.Button.X)
//                    .whenPressed(new LiftGoToPositionCommand(this, LiftPosition.Default.DOWN_DONT_DROP));
        }

        this.isTeleOp = teleOp;
        this.telemetry = telemetry;
    }

    public OldLift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug, Telemetry telemetry) {
        this(hardwareMap, toolGamepad, debug, true, telemetry);
    }

    public OldLift(HardwareMap hardwareMap, boolean debug) {
        this(hardwareMap, null, debug, null);
    }

    public OldLift(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public OldLift(HardwareMap hardwareMap, GamepadEx toolGamepad) {
        this(hardwareMap, toolGamepad, false, null);
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
            liftMotor.getCurrentPosition();
            if(liftMotor.getCurrentPosition() < MIN_LIFT_POSITION && toolGamepad.getLeftY() <= 0) {
                if (runMode != Motor.RunMode.VelocityControl) {
                    liftMotor.setRunMode(Motor.RunMode.VelocityControl);
                    runMode = Motor.RunMode.VelocityControl;
                }
                liftMotor.setTargetPosition(MIN_LIFT_POSITION);
                liftMotor.set(LIFT_POWER);
            } else if(liftMotor.getCurrentPosition() > MAX_LIFT_POSITION && toolGamepad.getLeftY() >= 0) {
                if (runMode != Motor.RunMode.VelocityControl) {
                    liftMotor.setRunMode(Motor.RunMode.VelocityControl);
                    runMode = Motor.RunMode.VelocityControl;
                }
                liftMotor.setTargetPosition(MAX_LIFT_POSITION);
                liftMotor.set(LIFT_POWER);
            } else {
                if (runMode != Motor.RunMode.RawPower) {
                    liftMotor.setRunMode(Motor.RunMode.RawPower);
                    runMode = Motor.RunMode.RawPower;
                }
                if(((liftMotor.getCurrentPosition()-MIN_LIFT_POSITION) < 10 && toolGamepad.getLeftY() <= 0) &&
                        ((MAX_LIFT_POSITION-liftMotor.getCurrentPosition()) < 10 && toolGamepad.getLeftY() >= 0)) {
                    liftMotor.set(0);
                } else {
                    liftMotor.set(toolGamepad.getLeftY() * LIFT_POWER);
                }
            }
            lastClawRotation = clawRotationServo.getPosition();
            double position = clawRotationServo.getPosition() + (((LiftPosition.Default.ONE.servoPosition - LiftPosition.Default.DOWN.servoPosition) / clawManualSpeed) * toolGamepad.getRightY());
            position = Math.max(MAX_CLAW_POSITION, position);
            clawRotationServo.setPosition(position);
            return;
        }
        if(runMode != Motor.RunMode.PositionControl) {
            liftMotor.setRunMode(Motor.RunMode.PositionControl);
            runMode = Motor.RunMode.PositionControl;
            clawRotationServo.setPosition(lastClawRotation);
        }

        if(position == null) {
            if(state == State.STARTING_MOVE) {
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                liftMotor.set(LIFT_POWER);
                state = State.AT_POSITION;
            }
            return;
        }
        liftMotor.set(LIFT_POWER);
        switch(state) {
            case STARTING_MOVE:
                state = State.EARLY_RUN_CLAW;
                if(position.shouldForceStartOpen() && clawServo.getPosition() == CLAW_CLOSED_POSITION) {
                    timer.reset();
                    clawServo.setPosition(CLAW_OPEN_POSITION);
                }
            case EARLY_RUN_CLAW:
                if(position.shouldForceStartOpen() && !clawOpen && timer.milliseconds() < clawOpenTime) break;
                if(position.shouldForceStartOpen()) clawOpen = true;
                state = State.EARLY_ROTATE_CLAW;
                if(position.rotateClawEarly()) {
                    double newPosition = position.getClawRotation();
                    calculatedRotationTime = Math.abs(newPosition - clawRotationServo.getPosition()) * clawRotateRunTimeP;
                    clawRotationServo.setPosition(newPosition);
                    timer.reset();
                }
            case EARLY_ROTATE_CLAW:
                if(position.rotateClawEarly() && timer.milliseconds() < calculatedRotationTime) break;
                state = State.RUNNING_LIFT;
                liftMotor.setTargetPosition(Math.max(MIN_LIFT_POSITION, Math.min(MAX_LIFT_POSITION, position.getLiftTicks())));
                liftMotor.set(LIFT_POWER);
            case RUNNING_LIFT:
                if(!liftMotor.atTargetPosition()) break;
                state = State.ROTATING_CLAW;
                double newPosition = position.getClawRotation();
                calculatedRotationTime = Math.abs(newPosition - clawRotationServo.getPosition()) * clawRotateRunTimeP;
                clawRotationServo.setPosition(newPosition);
                timer.reset();
            case ROTATING_CLAW:
                if(timer.milliseconds() < calculatedRotationTime) break;
                if (position.isClawOpen() && isTeleOp && !toolGamepad.isDown(GamepadKeys.Button.Y)) { // wait until confirmed with y
                    break;
                }
                state = State.RUNNING_CLAW;
                clawServo.setPosition(position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION);
                timer.reset();
            case RUNNING_CLAW:
                if(position.isClawOpen() != clawOpen && timer.milliseconds() < clawOpenTime) break;
                clawOpen = position.isClawOpen();
                state = State.AT_POSITION;
            case AT_POSITION:
                LiftPosition next = position.getNextLiftPosition();
                if(next != null) {
                    setPosition(next);
                }
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

    public boolean getClawOpen() {
        return clawOpen;
    }

    public LiftPosition getPosition() {
        return position;
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
