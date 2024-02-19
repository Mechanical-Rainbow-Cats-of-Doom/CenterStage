package org.firstinspires.ftc.teamcode.tool;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Config
public class NewLift extends SubsystemBase {
    public interface LiftPosition {
        int getLiftTicks();
        boolean isArmOut();
        double getArmRoll();
        double getArmLength();
        boolean isClawOpen();
        Time getLiftMoveTime();
        Time getArmRollTime();
        Time getArmYawTime();
        Time getArmLengthTime();
        Time getClawTime();
        LiftPosition getNextLiftPosition();

        /**
         * Represents the point in time where something will happen. Very late requires a button
         * press before it starts.
         */
        enum Time {
            EARLY, NORMAL, LATE, VERY_LATE
        }


        enum Default implements LiftPosition {
            DOWN(0, false, 0, 0, false, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.EARLY, Time.EARLY),
            LOW(1000, false, 0, 0, true, Time.NORMAL, Time.NORMAL, Time.EARLY, Time.NORMAL, Time.VERY_LATE),
            MEDIUM(2000, false, 0, 0, true, Time.NORMAL, Time.NORMAL, Time.EARLY, Time.NORMAL, Time.VERY_LATE),
            HIGH(3000, false, 0, 0, true, Time.NORMAL, Time.NORMAL, Time.EARLY, Time.NORMAL, Time.VERY_LATE);
            private final int liftTicks;
            private final boolean armOut;
            private final double armRoll;
            private final double armLength;
            private final Time liftMoveTime;
            private final Time armRollTime;
            private final Time armYawTime;
            private final Time armLengthTime;
            private final Time clawTime;
            private final boolean clawOpen;
            private final LiftPosition nextLiftPosition;

            Default(int liftTicks, boolean armOut, double armRoll, double armLength, boolean clawOpen,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime,
                    Time clawTime, LiftPosition nextLiftPosition) {
                this.liftTicks = liftTicks;
                this.armOut = armOut;
                this.armRoll = armRoll;
                this.armLength = armLength;
                this.clawOpen = clawOpen;
                this.liftMoveTime = liftMoveTime;
                this.armRollTime = armRollTime;
                this.armYawTime = armYawTime;
                this.armLengthTime = armLengthTime;
                this.clawTime = clawTime;
                this.nextLiftPosition = nextLiftPosition;
            }

            Default(int liftTicks, boolean armOut, double armRoll, double armLength, boolean clawOpen,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime,
                    Time clawTime) {
                this(liftTicks, armOut, armRoll, armLength, clawOpen, liftMoveTime, armRollTime,
                        armYawTime, armLengthTime, clawTime, null);
            }


            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public boolean isArmOut() {
                return armOut;
            }

            @Override
            public Time getLiftMoveTime() {
                return liftMoveTime;
            }

            @Override
            public double getArmRoll() {
                return armRoll;
            }

            @Override
            public double getArmLength() {
                return armLength;
            }

            @Override
            public Time getArmRollTime() {
                return armRollTime;
            }

            @Override
            public Time getArmYawTime() {
                return armYawTime;
            }

            @Override
            public Time getArmLengthTime() {
                return armLengthTime;
            }

            @Override
            public Time getClawTime() {
                return clawTime;
            }

            @Override
            public boolean isClawOpen() {
                return clawOpen;
            }

            @Override
            public LiftPosition getNextLiftPosition() {
                return nextLiftPosition;
            }
        }

        class Custom implements LiftPosition {
            private final int liftTicks;
            private final boolean armOut;
            private final double armRoll;
            private final double armLength;
            private final Time liftMoveTime;
            private final Time armRollTime;
            private final Time armYawTime;
            private final Time armLengthTime;
            private final Time clawTime;
            private final boolean clawOpen;
            private final LiftPosition nextLiftPosition;

            public Custom(int liftTicks, boolean armOut, double armRoll, double armLength, boolean clawOpen,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime,
                    Time clawTime, LiftPosition nextLiftPosition) {
                this.liftTicks = liftTicks;
                this.armOut = armOut;
                this.armRoll = armRoll;
                this.armLength = armLength;
                this.clawOpen = clawOpen;
                this.liftMoveTime = liftMoveTime;
                this.armRollTime = armRollTime;
                this.armYawTime = armYawTime;
                this.armLengthTime = armLengthTime;
                this.clawTime = clawTime;
                this.nextLiftPosition = nextLiftPosition;
            }

            public Custom(int liftTicks, boolean armOut, double armRoll, double armLength, boolean clawOpen,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime,
                    Time clawTime) {
                this(liftTicks, armOut, armRoll, armLength, clawOpen, liftMoveTime, armRollTime,
                        armYawTime, armLengthTime, clawTime, null);
            }


            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public boolean isArmOut() {
                return armOut;
            }

            @Override
            public Time getLiftMoveTime() {
                return liftMoveTime;
            }

            @Override
            public double getArmRoll() {
                return armRoll;
            }

            @Override
            public double getArmLength() {
                return armLength;
            }

            @Override
            public Time getArmRollTime() {
                return armRollTime;
            }

            @Override
            public Time getArmYawTime() {
                return armYawTime;
            }

            @Override
            public Time getArmLengthTime() {
                return armLengthTime;
            }

            @Override
            public LiftPosition getNextLiftPosition() {
                return nextLiftPosition;
            }

            @Override
            public boolean isClawOpen() {
                return clawOpen;
            }

            @Override
            public Time getClawTime() {
                return clawTime;
            }

        }
    }

    public enum State {
        STARTING_MOVE(false),
        EARLY_MOVE(false, LiftPosition.Time.EARLY),
        MOVE(false, LiftPosition.Time.NORMAL),
        LATE_MOVE(false, LiftPosition.Time.LATE),
        VERY_LATE_MOVE(false, LiftPosition.Time.VERY_LATE),
        AT_POSITION(true);

        public final boolean finished;
        public final LiftPosition.Time time;

        State(boolean finished, LiftPosition.Time time) {
            this.finished = finished;
            this.time = time;
        }

        State(boolean finished) {
            this(finished, null);
        }

        public boolean isFinished() {
            return finished;
        }

        public boolean currentTime(LiftPosition.Time time) {
            return time == this.time;
        }
    }

    public static double LIFT_POWER = 1;
    public static double kP = 1.2e-2;
    public static double POSITION_TOLERANCE = 15;
    public static int POSITION_LIMIT = 8196;
    public static double CLAW_OPEN_POSITION = 0;
    public static double CLAW_CLOSED_POSITION = 0;
    public static double ARM_IN_YAW = 0;
    public static double ARM_OUT_YAW = 0;
    public static double ARM_YAW_MOVE_TIME = 0;
    public static double CLAW_MOVE_TIME = 0;
    public static double ARM_ROLL_TIME_P = 0;
    public static double ARM_LENGTH_TIME_P = 0;
    private final MotorGroup liftMotor;
    private final ServoEx armYawServo;
    private final ServoEx armRollServo;
    private final ServoEx armLengthServo;
    private final ServoEx carriageRollServo;
    private final ServoEx carriageClawServo;
    private final GamepadEx toolGamepad;
    private LiftPosition position = LiftPosition.Default.DOWN;
    private State state = State.AT_POSITION;
    private boolean automatic = true;
    private Motor.RunMode runMode = Motor.RunMode.PositionControl;
    private double lastKP = kP;
    private double lastPositionTolerance = POSITION_TOLERANCE;
    private boolean waitingForLiftMove = false;
    private double maxMoveTime = 0;
    private ElapsedTime time = new ElapsedTime();

    private final boolean isTeleOp;

    @SuppressWarnings("ConstantConditions")
    public NewLift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug, boolean teleOp) {
        final Motor.GoBILDA motorType = Motor.GoBILDA.RPM_312;
        final String[] liftMotorIds = {"lift1", "lift2"};
        final boolean[] isInverted = {false, false};
        this.toolGamepad = toolGamepad;
        Motor[] motors = new Motor[liftMotorIds.length];
        for(int i = 0; i < liftMotorIds.length; i++) {
            motors[i] = new Motor(hardwareMap, liftMotorIds[i], motorType);
            motors[i].setInverted(isInverted[i]);
        }
        liftMotor = new MotorGroup(motors[0], motors.length > 1 ? Arrays.copyOfRange(motors, 1, motors.length) : null);
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.setInverted(true);
        liftMotor.resetEncoder();
        liftMotor.setPositionCoefficient(kP);
        liftMotor.setPositionTolerance(POSITION_TOLERANCE);
        liftMotor.setRunMode(Motor.RunMode.PositionControl);

        armYawServo = new SimpleServo(hardwareMap, "armYawServo", 0, 1);
        armRollServo = new SimpleServo(hardwareMap, "armRollServo", 0, 1);
        armLengthServo = new SimpleServo(hardwareMap, "armLengthServo", 0, 1);
        carriageRollServo = new SimpleServo(hardwareMap, "carriageRollServo", 0, 1);
        carriageClawServo = new SimpleServo(hardwareMap, "armYawServo", 0, 1);

        // TODO reimplement inputs
//        if (toolGamepad != null) {
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
//        }

        this.isTeleOp = teleOp;
    }

    public NewLift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug) {
        this(hardwareMap, toolGamepad, debug, true);
    }

    public NewLift(HardwareMap hardwareMap, boolean debug) {
        this(hardwareMap, null, debug);
    }

    public NewLift(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public NewLift(HardwareMap hardwareMap, GamepadEx toolGamepad) {
        this(hardwareMap, toolGamepad, false);
    }

    @Override
    public void periodic() {
        if(kP != lastKP) {
            liftMotor.setPositionCoefficient(kP);
            lastKP = kP;
        }
        if(POSITION_TOLERANCE != lastPositionTolerance) {
            liftMotor.setPositionTolerance(POSITION_TOLERANCE);
            lastPositionTolerance = POSITION_TOLERANCE;
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
        liftMotor.set(LIFT_POWER);
        switch(state) {
            case STARTING_MOVE:
                state = State.EARLY_MOVE;
                if(state.currentTime(position.getLiftMoveTime())) {
                    liftMotor.setTargetPosition(position.getLiftTicks());
                    waitingForLiftMove = true;
                }
                if(state.currentTime(position.getArmLengthTime())) {
                    double newArmLength = position.getArmLength();
                    double timeRequired = ARM_LENGTH_TIME_P * Math.abs(armLengthServo.getPosition() - newArmLength);
                    maxMoveTime = Math.max(maxMoveTime, timeRequired);
                    armLengthServo.setPosition(newArmLength);
                }
                if(state.currentTime(position.getArmRollTime())) {
                    double newArmRoll = position.getArmRoll();
                    double timeRequired = ARM_ROLL_TIME_P * Math.abs(armRollServo.getPosition() - newArmRoll);
                    maxMoveTime = Math.max(maxMoveTime, timeRequired);
                    armRollServo.setPosition(position.getArmRoll());
                    carriageRollServo.setPosition(-position.getArmRoll());
                }
                if(state.currentTime(position.getArmYawTime())) {
                    double requiredPosition = position.isArmOut() ? ARM_OUT_YAW : ARM_IN_YAW;
                    if(armYawServo.getPosition() != requiredPosition) {
                        maxMoveTime = Math.max(maxMoveTime, ARM_YAW_MOVE_TIME);
                        armYawServo.setPosition(requiredPosition);
                    }
                }
                if(state.currentTime(position.getClawTime())) {
                    double requiredPosition = position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
                    if(carriageClawServo.getPosition() != requiredPosition) {
                        maxMoveTime = Math.max(maxMoveTime, CLAW_MOVE_TIME);
                        carriageClawServo.setPosition(requiredPosition);
                    }
                }
                time.reset();
            case EARLY_MOVE:
                if(waitingForLiftMove && !liftMotor.atTargetPosition()) {
                    break;
                }
                if(maxMoveTime > time.seconds()) {
                    break;
                }
                waitingForLiftMove = false;
                maxMoveTime = 0;
                state = State.MOVE;
                if(state.currentTime(position.getLiftMoveTime())) {
                    liftMotor.setTargetPosition(position.getLiftTicks());
                    waitingForLiftMove = true;
                }
                if(state.currentTime(position.getArmLengthTime())) {
                    double newArmLength = position.getArmLength();
                    double timeRequired = ARM_LENGTH_TIME_P * Math.abs(armLengthServo.getPosition() - newArmLength);
                    maxMoveTime = Math.max(maxMoveTime, timeRequired);
                    armLengthServo.setPosition(newArmLength);
                }
                if(state.currentTime(position.getArmRollTime())) {
                    double newArmRoll = position.getArmRoll();
                    double timeRequired = ARM_ROLL_TIME_P * Math.abs(armRollServo.getPosition() - newArmRoll);
                    maxMoveTime = Math.max(maxMoveTime, timeRequired);
                    armRollServo.setPosition(position.getArmRoll());
                    carriageRollServo.setPosition(-position.getArmRoll());
                }
                if(state.currentTime(position.getArmYawTime())) {
                    double requiredPosition = position.isArmOut() ? ARM_OUT_YAW : ARM_IN_YAW;
                    if(armYawServo.getPosition() != requiredPosition) {
                        maxMoveTime = Math.max(maxMoveTime, ARM_YAW_MOVE_TIME);
                        armYawServo.setPosition(requiredPosition);
                    }
                }
                if(state.currentTime(position.getClawTime())) {
                    double requiredPosition = position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
                    if(carriageClawServo.getPosition() != requiredPosition) {
                        maxMoveTime = Math.max(maxMoveTime, CLAW_MOVE_TIME);
                        carriageClawServo.setPosition(requiredPosition);
                    }
                }
                time.reset();
            case MOVE:
                if(waitingForLiftMove && !liftMotor.atTargetPosition()) {
                    break;
                }
                if(maxMoveTime > time.seconds()) {
                    break;
                }
                waitingForLiftMove = false;
                maxMoveTime = 0;
                state = State.LATE_MOVE;
                if(state.currentTime(position.getLiftMoveTime())) {
                    liftMotor.setTargetPosition(position.getLiftTicks());
                    waitingForLiftMove = true;
                }
                if(state.currentTime(position.getArmLengthTime())) {
                    double newArmLength = position.getArmLength();
                    double timeRequired = ARM_LENGTH_TIME_P * Math.abs(armLengthServo.getPosition() - newArmLength);
                    maxMoveTime = Math.max(maxMoveTime, timeRequired);
                    armLengthServo.setPosition(newArmLength);
                }
                if(state.currentTime(position.getArmRollTime())) {
                    double newArmRoll = position.getArmRoll();
                    double timeRequired = ARM_ROLL_TIME_P * Math.abs(armRollServo.getPosition() - newArmRoll);
                    maxMoveTime = Math.max(maxMoveTime, timeRequired);
                    armRollServo.setPosition(position.getArmRoll());
                    carriageRollServo.setPosition(-position.getArmRoll());
                }
                if(state.currentTime(position.getArmYawTime())) {
                    double requiredPosition = position.isArmOut() ? ARM_OUT_YAW : ARM_IN_YAW;
                    if(armYawServo.getPosition() != requiredPosition) {
                        maxMoveTime = Math.max(maxMoveTime, ARM_YAW_MOVE_TIME);
                        armYawServo.setPosition(requiredPosition);
                    }
                }
                if(state.currentTime(position.getClawTime())) {
                    double requiredPosition = position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
                    if(carriageClawServo.getPosition() != requiredPosition) {
                        maxMoveTime = Math.max(maxMoveTime, CLAW_MOVE_TIME);
                        carriageClawServo.setPosition(requiredPosition);
                    }
                }
                time.reset();
            case LATE_MOVE:
                if(waitingForLiftMove && !liftMotor.atTargetPosition()) {
                    break;
                }
                if(maxMoveTime > time.seconds()) {
                    break;
                }
                waitingForLiftMove = false;
                maxMoveTime = 0;
                if(!usesTime(position, LiftPosition.Time.VERY_LATE)) {
                    state = State.AT_POSITION;
                } else {
                    if (!toolGamepad.getButton(GamepadKeys.Button.Y)) {
                        break;
                    }
                    state = State.VERY_LATE_MOVE;
                    if(state.currentTime(position.getLiftMoveTime())) {
                        liftMotor.setTargetPosition(position.getLiftTicks());
                        waitingForLiftMove = true;
                    }
                    if(state.currentTime(position.getArmLengthTime())) {
                        double newArmLength = position.getArmLength();
                        double timeRequired = ARM_LENGTH_TIME_P * Math.abs(armLengthServo.getPosition() - newArmLength);
                        maxMoveTime = Math.max(maxMoveTime, timeRequired);
                        armLengthServo.setPosition(newArmLength);
                    }
                    if(state.currentTime(position.getArmRollTime())) {
                        double newArmRoll = position.getArmRoll();
                        double timeRequired = ARM_ROLL_TIME_P * Math.abs(armRollServo.getPosition() - newArmRoll);
                        maxMoveTime = Math.max(maxMoveTime, timeRequired);
                        armRollServo.setPosition(position.getArmRoll());
                        carriageRollServo.setPosition(-position.getArmRoll());
                    }
                    if(state.currentTime(position.getArmYawTime())) {
                        double requiredPosition = position.isArmOut() ? ARM_OUT_YAW : ARM_IN_YAW;
                        if(armYawServo.getPosition() != requiredPosition) {
                            maxMoveTime = Math.max(maxMoveTime, ARM_YAW_MOVE_TIME);
                            armYawServo.setPosition(requiredPosition);
                        }
                    }
                    if(state.currentTime(position.getClawTime())) {
                        double requiredPosition = position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
                        if(carriageClawServo.getPosition() != requiredPosition) {
                            maxMoveTime = Math.max(maxMoveTime, CLAW_MOVE_TIME);
                            carriageClawServo.setPosition(requiredPosition);
                        }
                    }
                }
            case VERY_LATE_MOVE:
                if(waitingForLiftMove && !liftMotor.atTargetPosition()) {
                    break;
                }
                if(maxMoveTime > time.seconds()) {
                    break;
                }
                waitingForLiftMove = false;
                maxMoveTime = 0;
                state = State.AT_POSITION;
            case AT_POSITION:
                break;
        }
    }

    public int getCurrentLiftPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void setPosition(LiftPosition position) {
        this.position = position;
        this.state = State.STARTING_MOVE;
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

    public double getPower() {
        return liftMotor.get();
    }

    private static boolean usesTime(@NonNull LiftPosition position, LiftPosition.Time time) {
        return position.getLiftMoveTime() == time || position.getClawTime() == time ||
                position.getArmLengthTime() == time || position.getArmYawTime() == time ||
                position.getArmRollTime() == time;
    }
}
