package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

@Config
public class NewLift extends SubsystemBase {
    public interface LiftPosition {
        int getLiftTicks();
        boolean isArmOut();
        double getArmRoll();
        double getArmLength();
        Time getLiftMoveTime();
        Time getArmRollTime();
        Time getArmYawTime();
        Time getArmLengthTime();
        LiftPosition getNextLiftPosition();

        enum Time {
            EARLY, NORMAL, LATE
        }


        enum Default implements LiftPosition {
            DOWN(0, false, 0, 0, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.EARLY);
            private final int liftTicks;
            private final boolean armOut;
            private final double armRoll;
            private final double armLength;
            private final Time liftMoveTime;
            private final Time armRollTime;
            private final Time armYawTime;
            private final Time armLengthTime;
            private final LiftPosition nextLiftPosition;

            Default(int liftTicks, boolean armOut, double armRoll, double armLength,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime,
                    LiftPosition nextLiftPosition) {
                this.liftTicks = liftTicks;
                this.armOut = armOut;
                this.armRoll = armRoll;
                this.armLength = armLength;
                this.liftMoveTime = liftMoveTime;
                this.armRollTime = armRollTime;
                this.armYawTime = armYawTime;
                this.armLengthTime = armLengthTime;
                this.nextLiftPosition = nextLiftPosition;
            }

            Default(int liftTicks, boolean armOut, double armRoll, double armLength,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime) {
                this(liftTicks, armOut, armRoll, armLength, liftMoveTime, armRollTime, armYawTime,
                        armLengthTime, null);
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
            private final LiftPosition nextLiftPosition;

            public Custom(int liftTicks, boolean armOut, double armRoll, double armLength,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime,
                    LiftPosition nextLiftPosition) {
                this.liftTicks = liftTicks;
                this.armOut = armOut;
                this.armRoll = armRoll;
                this.armLength = armLength;
                this.liftMoveTime = liftMoveTime;
                this.armRollTime = armRollTime;
                this.armYawTime = armYawTime;
                this.armLengthTime = armLengthTime;
                this.nextLiftPosition = nextLiftPosition;
            }

            public Custom(int liftTicks, boolean armOut, double armRoll, double armLength,
                    Time liftMoveTime, Time armRollTime, Time armYawTime, Time armLengthTime) {
                this(liftTicks, armOut, armRoll, armLength, liftMoveTime, armRollTime, armYawTime,
                        armLengthTime, null);
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
        }
    }

    public enum State {
        STARTING_MOVE(false, false),
        EARLY_MOVE(false, false, LiftPosition.Time.EARLY),
        MOVE(false, false, LiftPosition.Time.NORMAL),
        LATE_MOVE(false, false, LiftPosition.Time.LATE),
        AT_POSITION(true, true);

        public final boolean liftMoved;
        public final boolean finished;
        public final LiftPosition.Time time;

        State(boolean liftMoved, boolean finished, LiftPosition.Time time) {
            this.liftMoved = liftMoved;
            this.finished = finished;
            this.time = time;
        }

        State(boolean liftMoved, boolean finished) {
            this(liftMoved, finished, null);
        }

        public boolean isLiftMoved() {
            return liftMoved;
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
    public static double positionTolerence = 20;
    public static int positionLimit = 8196;
    private final MotorGroup liftMotor;
    private final ServoEx armYawServo;
    private final ServoEx armRollServo;
    private final ServoEx armLengthServo;
    private final ServoEx carriageRollServo;
    private final ServoEx carriageDropServo;
    private final GamepadEx toolGamepad;
    private LiftPosition position = LiftPosition.Default.DOWN;
    private State state = State.AT_POSITION;
    private boolean clawOpen = false;
    private boolean automatic = true;
    private Motor.RunMode runMode = Motor.RunMode.PositionControl;
    private double lastKP = kP;
    private double lastPositionTolerence = positionTolerence;

    private final boolean isTeleOp;

    public NewLift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug, boolean teleOp) {
        final Motor.GoBILDA motorType = Motor.GoBILDA.RPM_312;
        final String[] liftMotorIds = {"lift1", "lift2"};
        this.toolGamepad = toolGamepad;
        Motor[] motors = new Motor[liftMotorIds.length];
        for(int i = 0; i < liftMotorIds.length; i++) {
            motors[i] = new Motor(hardwareMap, liftMotorIds[i], motorType);
        }
        //noinspection ConstantValue
        liftMotor = new MotorGroup(motors[0], motors.length > 1 ? Arrays.copyOfRange(motors, 1, motors.length) : null);
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.setInverted(true);
        liftMotor.resetEncoder();
        liftMotor.setPositionCoefficient(kP);
        liftMotor.setPositionTolerance(positionTolerence);
        liftMotor.setRunMode(Motor.RunMode.PositionControl);


        armYawServo = new SimpleServo(hardwareMap, "armYawServo", 0, 1);
        armRollServo = new SimpleServo(hardwareMap, "armRollServo", 0, 1);
        armLengthServo = new SimpleServo(hardwareMap, "armLengthServo", 0, 1);
        carriageRollServo = new SimpleServo(hardwareMap, "carriageRollServo", 0, 1);
        carriageDropServo = new SimpleServo(hardwareMap, "armYawServo", 0, 1);

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
        liftMotor.set(LIFT_POWER);
        switch(state) {
            case STARTING_MOVE:
                state = State.EARLY_MOVE;
            case EARLY_MOVE:
                if(state.currentTime(position.getLiftMoveTime())) {

                }
                state = State.MOVE;
            case MOVE:

                state = State.LATE_MOVE;
            case LATE_MOVE:

                state = State.AT_POSITION;
            case AT_POSITION:

                break;
        }
    }

    public int getCurrentLiftPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void toggleClawOpen() {
        if(state != State.AT_POSITION) setPosition(null);
        clawOpen = !clawOpen;
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
}
