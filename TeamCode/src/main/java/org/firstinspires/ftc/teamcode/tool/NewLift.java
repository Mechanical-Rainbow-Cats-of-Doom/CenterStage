package org.firstinspires.ftc.teamcode.tool;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LiftGoToPositionCommand;

import java.util.Arrays;

@Config
public class NewLift extends SubsystemBase {
    private final SimpleServo armYawRight;

    public interface LiftPosition {
        int getLiftTicks();
        int getLiftTicksTwo();
        boolean isArmOut();
        /**
         * left is 0, middle is 0.43, right is 0.8
         */
        double getArmRoll();
        /**
         * left is closer to 0
         */
        double getCarriageRoll();
        /**
         * 0 shorter 1 longer
         * range: 1-0.35
         */
        double getArmLength();
        boolean isClawOpen();
        boolean retractArm();
        Time getLiftMoveTime();
        Time getLiftMoveTwoTime();
        Time getArmYawTime();
        Time getArmRollTime();
        Time getCarriageRollTime();
        Time getArmLengthTime();
        Time getClawTime();
        Time getRetractArmTime();
        LiftPosition getNextLiftPosition();

        /**
         * Represents the point in time where something will happen. Very late requires a button
         * press before it starts.
         */
        enum Time {
            VERY_EARLY, EARLY, NORMAL, LATE, VERY_LATE_BUTTON
        }


        enum Default implements LiftPosition {
            DOWN(0, 100, false, 0.43, 0.535, 0.9, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.VERY_EARLY, Time.VERY_EARLY, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY),


            T_LEFT_LOW(800, 100, true, 0.76, 0.76, 0.35, true, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY, Default.DOWN),
            T_LEFT_MEDIUM(1400, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_LEFT_LOW.armRoll, T_LEFT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),
            T_LEFT_HIGH(2050, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_LEFT_LOW.armRoll, T_LEFT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),

            T_MIDDLE_LOW(400, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),
            T_MIDDLE_MEDIUM(1000, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),
            T_MIDDLE_HIGH(1650, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),

            T_RIGHT_LOW(800, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.14, 0.33, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),
            T_RIGHT_MEDIUM(1400, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_RIGHT_LOW.armRoll, T_RIGHT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),
            T_RIGHT_HIGH(2050, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_RIGHT_LOW.armRoll, T_RIGHT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armYawTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.nextLiftPosition),

            // NO WORK
            A_VLOW_MIDDLEDUMP_RIGHT_LEAN(0, 100, true, 0.7, 0.535, 1, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),
            // NO WORK
            A_VLOW_MIDDLEDUMP_LEFT_LEAN(0, 100, true, 0.7, 0.535, 1, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),
            A_MIDDLE_VLOW_LEFTDUMP(200, 100, true, 0.25, 0.3, 1, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),
            A_MIDDLE_VLOW_RIGHTDUMP(100, 100, true, 0.5, 0.75, 1, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),

            A_LEFT_VLOW_LEFTDUMP(100, 100, true, 0.5, 0.75, 0.35, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),
            A_LEFT_VLOW_MIDDLEDUMP(300, 100, true, 0.8, 1, 0.35, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),

            A_RIGHT_VLOW_RIGHTDUMP(100, 100, true, 0.36, 0.32, 0.35, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),
            A_RIGHT_VLOW_MIDDLEDUMP(300, 100, true, 0.06, 0.07, 0.35, false, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY),


            A_MEDIUM(2000, 100, true, 0.43, 0.535, 1, true, true,
                     Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.EARLY, Time.LATE, Time.VERY_LATE_BUTTON, Time.EARLY),
            A_HIGH(3000, 100, true, 0.43, 0.535, 1, true, true,
                   Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.EARLY, Time.LATE, Time.VERY_LATE_BUTTON, Time.EARLY);

            private final int liftTicks;
            private final int liftTicksTwo;
            private final boolean armOut;
            private final double armRoll;
            private final double carriageRoll;
            private final double armLength;
            private final boolean retractArm;
            private final Time liftMoveTime;
            private final Time liftMoveTwoTime;
            private final Time armRollTime;
            private final Time carriageRollTime;
            private final Time armYawTime;
            private final Time armLengthTime;
            private final Time clawTime;
            private final Time retractArmTime;
            private final boolean clawOpen;
            private final LiftPosition nextLiftPosition;

            Default(int liftTicks, int liftTicksTwo, boolean armOut, double armRoll, double carriageRoll,
                    double armLength, boolean clawOpen, boolean retractArm,
                    Time liftMoveTime, Time liftMoveTwoTime, Time armYawTime, Time armRollTime, Time carriageRollTime,
                    Time armLengthTime, Time clawTime, Time retractArmTime,
                    LiftPosition nextLiftPosition) {
                this.liftTicks = liftTicks;
                this.liftTicksTwo = liftTicksTwo;
                this.armOut = armOut;
                this.armRoll = armRoll;
                this.carriageRoll = carriageRoll;
                this.armLength = armLength;
                this.clawOpen = clawOpen;
                this.liftMoveTime = liftMoveTime;
                this.liftMoveTwoTime = liftMoveTwoTime;
                this.armRollTime = armRollTime;
                this.carriageRollTime = carriageRollTime;
                this.armYawTime = armYawTime;
                this.armLengthTime = armLengthTime;
                this.clawTime = clawTime;
                this.nextLiftPosition = nextLiftPosition;
                this.retractArm = retractArm;
                this.retractArmTime = retractArmTime;
            }

            Default(int liftTicks, int liftTicksTwo, boolean armOut, double armRoll, double carriageRoll,
                    double armLength, boolean clawOpen, boolean retractArm,
                    Time liftMoveTime, Time liftMoveTwoTime, Time armYawTime, Time armRollTime, Time carriageRollTime,
                    Time armLengthTime, Time clawTime, Time retractArmTime) {
                this(liftTicks, liftTicksTwo, armOut, armRoll, carriageRoll, armLength, clawOpen, retractArm,
                        liftMoveTime, liftMoveTwoTime, armYawTime, armRollTime, carriageRollTime,
                        armLengthTime, clawTime, retractArmTime, null);
            }


            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public int getLiftTicksTwo() {
                return liftTicksTwo;
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
            public Time getLiftMoveTwoTime() {
                return liftMoveTwoTime;
            }

            @Override
            public double getArmRoll() {
                return armRoll;
            }

            @Override
            public double getCarriageRoll() {
                return carriageRoll;
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
            public Time getCarriageRollTime() {
                return carriageRollTime;
            }

            @Override
            public boolean isClawOpen() {
                return clawOpen;
            }

            @Override
            public boolean retractArm() {
                return retractArm;
            }

            @Override
            public Time getRetractArmTime() {
                return retractArmTime;
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
            private final double carriageRoll;
            private final double armLength;
            private final boolean retractArm;
            private final Time liftMoveTime;
            private final Time armRollTime;
            private final Time carriageRollTime;
            private final Time armYawTime;
            private final Time armLengthTime;
            private final Time clawTime;
            private final Time retractArmTime;
            private final boolean clawOpen;
            private final LiftPosition nextLiftPosition;

            public Custom(int liftTicks, boolean armOut, double armRoll, double carriageRoll,
                    double armLength, boolean clawOpen, boolean retractArm, Time liftMoveTime,
                    Time armRollTime, Time carriageRollTime, Time armYawTime, Time armLengthTime,
                    Time clawTime, Time retractArmTime, LiftPosition nextLiftPosition) {
                this.liftTicks = liftTicks;
                this.armOut = armOut;
                this.armRoll = armRoll;
                this.carriageRoll = carriageRoll;
                this.armLength = armLength;
                this.clawOpen = clawOpen;
                this.liftMoveTime = liftMoveTime;
                this.armRollTime = armRollTime;
                this.carriageRollTime = carriageRollTime;
                this.armYawTime = armYawTime;
                this.armLengthTime = armLengthTime;
                this.clawTime = clawTime;
                this.nextLiftPosition = nextLiftPosition;
                this.retractArm = retractArm;
                this.retractArmTime = retractArmTime;
            }

            public Custom(int liftTicks, boolean armOut, double armRoll, double carriageRoll,
                    double armLength, boolean clawOpen, boolean retractArm, Time liftMoveTime,
                    Time armRollTime, Time carriageRollTime, Time armYawTime, Time armLengthTime,
                    Time clawTime, Time retractArmTime) {
                this(liftTicks, armOut, armRoll, carriageRoll, armLength, clawOpen, retractArm,
                        liftMoveTime, armRollTime, carriageRollTime, armYawTime, armLengthTime,
                        clawTime, retractArmTime, null);
            }


            @Override
            public int getLiftTicks() {
                return liftTicks;
            }

            @Override
            public int getLiftTicksTwo() {
                return 0;
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
            public Time getLiftMoveTwoTime() {
                return null;
            }

            @Override
            public double getArmRoll() {
                return armRoll;
            }

            @Override
            public double getCarriageRoll() {
                return carriageRoll;
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
            public Time getCarriageRollTime() {
                return carriageRollTime;
            }

            @Override
            public boolean isClawOpen() {
                return clawOpen;
            }

            @Override
            public boolean retractArm() {
                return retractArm;
            }

            @Override
            public Time getRetractArmTime() {
                return retractArmTime;
            }

            @Override
            public LiftPosition getNextLiftPosition() {
                return nextLiftPosition;
            }
        }
    }

    public enum State {
        STARTING_MOVE(false),
        VERY_EARLY_MOVE(false, LiftPosition.Time.VERY_EARLY),
        EARLY_MOVE(false, LiftPosition.Time.EARLY),
        MOVE(false, LiftPosition.Time.NORMAL),
        LATE_MOVE(false, LiftPosition.Time.LATE),
        VERY_LATE_BUTTON_MOVE(false, LiftPosition.Time.VERY_LATE_BUTTON),
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
    public static int POSITION_LIMIT = 3000;
    public static double CLAW_OPEN_POSITION = 0;
    public static double CLAW_CLOSED_POSITION = 0.2;
    public static double ARM_IN_YAW = 0.34;
    public static double ARM_OUT_YAW = 0.44;
    public static double ARM_YAW_MOVE_TIME = 1.33;
    public static double CLAW_MOVE_TIME = 0.2;
    public static double ARM_ROLL_TIME_P = 0.79;
    public static double CARRIAGE_ROLL_TIME_P = 0.6;
    public static double ARM_LENGTH_TIME_P = 1.3;
    public static double FIRST_SERVO_MIDDLE = 0.5;
    public static double SECOND_SERVO_MIDDLE = 0.5;
    public static boolean SECOND_SERVO_INVERTED = true;
    public static double ARM_RETRACTED_POSITION = 1;
    public static double MIN_CARRIAGE_ANGLE = 0;
    public static double MAX_CARRIAGE_ANGLE = 1;
    public static double CARRIAGE_ANGLE_OFFSET = 0;

    public int direction = 0;
    public int height = 0;

    private final MotorGroup liftMotor;
    private final Motor liftLeader;
    private final ServoEx armYawLeft;
    private final ServoEx armRollServo;
    private final ServoEx armLengthServo;
    private final ServoEx carriageRollServo;
    private final ServoEx carriageClawServo;
    private final GamepadEx toolGamepad;
    // the blursed method
    private LiftPosition position = new LiftPosition() {
        @Override
        public int getLiftTicks() {
            return 0;
        }

        @Override
        public int getLiftTicksTwo() {
            return 0;
        }

        @Override
        public boolean isArmOut() {
            return false;
        }

        @Override
        public double getArmRoll() {
            return 0.43;
        }

        @Override
        public double getCarriageRoll() {
            return 0.535;
        }

        @Override
        public double getArmLength() {
            return 0.9;
        }

        @Override
        public boolean isClawOpen() {
            return false;
        }

        @Override
        public boolean retractArm() {
            return false;
        }

        @Override
        public Time getLiftMoveTime() {
            return Time.NORMAL;
        }

        @Override
        public Time getLiftMoveTwoTime() {
            return null;
        }

        @Override
        public Time getArmYawTime() {
            return Time.NORMAL;
        }

        @Override
        public Time getArmRollTime() {
            return Time.NORMAL;
        }

        @Override
        public Time getCarriageRollTime() {
            return Time.NORMAL;
        }

        @Override
        public Time getArmLengthTime() {
            return Time.NORMAL;
        }

        @Override
        public Time getClawTime() {
            return null;
        }

        @Override
        public Time getRetractArmTime() {
            return null;
        }

        @Override
        public LiftPosition getNextLiftPosition() {
            return Default.DOWN;
        }
    };
    private LiftPosition previousLiftPosition = LiftPosition.Default.DOWN;
    private State state = State.STARTING_MOVE;
    private boolean automatic = true;
    private Motor.RunMode runMode = Motor.RunMode.PositionControl;
    private boolean waitingForLiftMove = false;
    private double maxMoveTime = 0;
    private final ElapsedTime time = new ElapsedTime();
    private boolean initial = true;

    private double lastKP = kP;
    private double lastPositionTolerance = POSITION_TOLERANCE;
    private double lastMinCarriageAngle = MIN_CARRIAGE_ANGLE;
    private double lastMaxCarriageAngle = MAX_CARRIAGE_ANGLE;

    private final boolean isTeleOp;

    @SuppressWarnings("ConstantConditions")
    public NewLift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug, boolean teleOp) {
        final Motor.GoBILDA motorType = Motor.GoBILDA.RPM_312;
        final String[] liftMotorIds = {"liftRight", "liftLeft"};
        final boolean[] isInverted = {false, true};
        this.toolGamepad = toolGamepad;
        Motor[] motors = new Motor[liftMotorIds.length];
        for(int i = 0; i < liftMotorIds.length; i++) {
            motors[i] = new Motor(hardwareMap, liftMotorIds[i], motorType);
            motors[i].setInverted(isInverted[i]);
        }
        liftLeader = motors[0];
        liftMotor = new MotorGroup(motors[0], motors.length > 1 ? Arrays.copyOfRange(motors, 1, motors.length) : null);
        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftMotor.resetEncoder();
        liftMotor.setPositionCoefficient(kP);
        liftMotor.setPositionTolerance(POSITION_TOLERANCE);
        liftMotor.setRunMode(Motor.RunMode.PositionControl);

        armYawLeft = new SimpleServo(hardwareMap, "armYawLeft", 0, 1);
        armYawRight = new SimpleServo(hardwareMap, "armYawRight", 0, 1);
        armRollServo = new SimpleServo(hardwareMap, "armRoll", 0, 1);
        armLengthServo = new SimpleServo(hardwareMap, "armLength", MIN_CARRIAGE_ANGLE, MAX_CARRIAGE_ANGLE);
        carriageRollServo = new SimpleServo(hardwareMap, "carriageRoll", 0, 1);
        carriageClawServo = new SimpleServo(hardwareMap, "carriageClaw", 0, 1);

        // TODO reimplement inputs
        if (toolGamepad != null) {
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new LiftGoToPositionCommand(this, LiftPosition.Default.DOWN));
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                    .whenPressed(new SelectPositionCommand(2, false));
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new SelectPositionCommand(1, false));
            toolGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                    .whenPressed(new SelectPositionCommand(0, false));
            toolGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(new SelectPositionCommand(-1, true));
            toolGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(new SelectPositionCommand(1, true));
        }

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
        if(lastMinCarriageAngle != MIN_CARRIAGE_ANGLE || lastMaxCarriageAngle != MAX_CARRIAGE_ANGLE) {
            carriageRollServo.setRange(MIN_CARRIAGE_ANGLE, MAX_CARRIAGE_ANGLE);
            lastMinCarriageAngle = MIN_CARRIAGE_ANGLE;
            lastMaxCarriageAngle = MAX_CARRIAGE_ANGLE;
        }
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
            double angle = Math.atan2(toolGamepad.getRightY(), toolGamepad.getRightX());
            carriageRollServo.turnToAngle(angle);
            return;
        }
        if(runMode != Motor.RunMode.PositionControl) {
            liftMotor.setRunMode(Motor.RunMode.PositionControl);
            runMode = Motor.RunMode.PositionControl;
        }

        liftMotor.set(1);
        switch(state) {
            case STARTING_MOVE:
                state = State.VERY_EARLY_MOVE;
                waitingForLiftMove = false;
                maxMoveTime = 0;
                if(previousLiftPosition.isArmOut() != position.isArmOut()) {
                    prepareMove();
                    time.reset();
                }
            case VERY_EARLY_MOVE:
                if(previousLiftPosition.isArmOut() != position.isArmOut() || initial) {
                    if (waitingForLiftMove && !liftMotor.atTargetPosition()) {
                        break;
                    }
                    if (maxMoveTime > time.seconds()) {
                        break;
                    }
                    waitingForLiftMove = false;
                    maxMoveTime = 0;
                    state = State.EARLY_MOVE;
                    prepareMove();
                    time.reset();
                }
            case EARLY_MOVE:
                if(previousLiftPosition.isArmOut() != position.isArmOut()) {
                    if (waitingForLiftMove && !liftMotor.atTargetPosition()) {
                        break;
                    }
                    if (maxMoveTime > time.seconds()) {
                        break;
                    }
                    waitingForLiftMove = false;
                    maxMoveTime = 0;
                }
                state = State.MOVE;
                prepareMove();
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
                prepareMove();
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
                if(!usesTime(position, LiftPosition.Time.VERY_LATE_BUTTON)) {
                    state = State.AT_POSITION;
                } else {
                    if (!toolGamepad.getButton(GamepadKeys.Button.Y)) {
                        break;
                    }
                    state = State.VERY_LATE_BUTTON_MOVE;
                    prepareMove();
                    maxMoveTime = Math.min(0.2, maxMoveTime);
                }
            case VERY_LATE_BUTTON_MOVE:
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
                LiftPosition next = position.getNextLiftPosition();
                if(next != null && toolGamepad.getButton(GamepadKeys.Button.Y)) {
                    setPosition(next);
                }
                break;
        }
    }

    public int getCurrentLiftPosition() {
        return liftLeader.getCurrentPosition();
    }

    public void setPosition(LiftPosition position) {
        if(position == null) return;
        initial = false;
        previousLiftPosition = this.position;
        this.position = position;
        state = State.STARTING_MOVE;
    }

    public LiftPosition getPosition() {
        return position;
    }

    public void toggleArmPosition() {
        if(!automatic) {
            double requiredPosition = (armYawRight.getPosition() == ARM_IN_YAW) ? ARM_OUT_YAW : ARM_IN_YAW;

            armYawRight.setPosition(requiredPosition);
            armYawLeft.setPosition((requiredPosition-FIRST_SERVO_MIDDLE)*(SECOND_SERVO_INVERTED ? -1 : 1)+SECOND_SERVO_MIDDLE);
        }
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
                position.getArmRollTime() == time || position.getCarriageRollTime() == time ||
                position.getRetractArmTime() == time || position.getLiftMoveTwoTime() == time;
    }

    private void prepareMove() {
        if(state.currentTime(position.getLiftMoveTime())) {
            liftMotor.setTargetPosition(Math.max(0, Math.min(position.getLiftTicks(), POSITION_LIMIT)));
            waitingForLiftMove = true;
        } else if(state.currentTime(position.getLiftMoveTwoTime())) {
            liftMotor.setTargetPosition(Math.max(0, Math.min(position.getLiftTicksTwo(), POSITION_LIMIT)));
            waitingForLiftMove = true;
        }
        if(state.currentTime(position.getArmLengthTime())) {
            double newArmLength = position.getArmLength();
            double timeRequired = ARM_LENGTH_TIME_P * Math.abs(armLengthServo.getPosition() - newArmLength);
            maxMoveTime = Math.max(maxMoveTime, timeRequired);
            armLengthServo.setPosition(newArmLength);
        } else if(position.retractArm() && state.currentTime(position.getRetractArmTime())) {
            double newArmLength = ARM_RETRACTED_POSITION;
            double timeRequired = ARM_LENGTH_TIME_P * Math.abs(armLengthServo.getPosition() - newArmLength);
            maxMoveTime = Math.max(maxMoveTime, timeRequired);
            armLengthServo.setPosition(newArmLength);
        }
        if(state.currentTime(position.getArmRollTime())) {
            double newArmRoll = position.getArmRoll();
            double timeRequired = ARM_ROLL_TIME_P * Math.abs(armRollServo.getPosition() - newArmRoll);
            maxMoveTime = Math.max(maxMoveTime, timeRequired);
            armRollServo.setPosition(position.getArmRoll());
        }
        if(state.currentTime(position.getCarriageRollTime())) {
            double newArmRoll = position.getCarriageRoll();
            double timeRequired = CARRIAGE_ROLL_TIME_P * Math.abs(carriageRollServo.getPosition() - newArmRoll);
            maxMoveTime = Math.max(maxMoveTime, timeRequired);
            carriageRollServo.setPosition(position.getCarriageRoll());
        }
        if(state.currentTime(position.getArmYawTime())) {
            double requiredPosition = position.isArmOut() ? ARM_OUT_YAW : ARM_IN_YAW;
            if(armYawRight.getPosition() != requiredPosition) {
                maxMoveTime = Math.max(maxMoveTime, ARM_YAW_MOVE_TIME);
                armYawRight.setPosition(requiredPosition);
                armYawLeft.setPosition((requiredPosition-FIRST_SERVO_MIDDLE)*(SECOND_SERVO_INVERTED ? -1 : 1)+SECOND_SERVO_MIDDLE);
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

    private class SelectPositionCommand extends CommandBase {
        private final int o;
        private final boolean height;

        private SelectPositionCommand(int o, boolean height) {
            this.o = o;
            this.height = height;
        }

        @Override
        public void execute() {
            if(height) {
                NewLift.this.height = Math.max(0, Math.min(NewLift.this.height + o, 2));
            } else {
                NewLift.this.direction = o;
            }

            chooseNewLiftPosition();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private void chooseNewLiftPosition() {
        direction = direction % 3;
        height = height % 3;
        setPosition(selectPosition(direction, height));
    }

    private static LiftPosition selectPosition(int direction, int height) {
        switch(direction) {
            case 0:
                switch(height) {
                    case 0:
                        return LiftPosition.Default.T_LEFT_LOW;
                    case 1:
                        return LiftPosition.Default.T_LEFT_MEDIUM;
                    case 2:
                        return LiftPosition.Default.T_LEFT_HIGH;
                    default:
                        return LiftPosition.Default.DOWN;
                }
            case 1:
                switch(height) {
                    case 0:
                        return LiftPosition.Default.T_MIDDLE_LOW;
                    case 1:
                        return LiftPosition.Default.T_MIDDLE_MEDIUM;
                    case 2:
                        return LiftPosition.Default.T_MIDDLE_HIGH;
                    default:
                        return LiftPosition.Default.DOWN;
                }
            case 2:
                switch(height) {
                    case 0:
                        return LiftPosition.Default.T_RIGHT_LOW;
                    case 1:
                        return LiftPosition.Default.T_RIGHT_MEDIUM;
                    case 2:
                        return LiftPosition.Default.T_RIGHT_HIGH;
                    default:
                        return LiftPosition.Default.DOWN;
                }
            default:
                return LiftPosition.Default.DOWN;
        }
    }

    public void setClaw(boolean open) {
        carriageClawServo.setPosition(open ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION);
    }

    public boolean isClawOpen() {
        return carriageClawServo.getPosition() != CLAW_CLOSED_POSITION;
    }

    private class ClawAction implements Action {
        private final boolean open;
        private boolean started = false;
        private ElapsedTime time = new ElapsedTime();

        public ClawAction(boolean open) {
            this.open = open;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!started) {
                if(carriageClawServo.getPosition() == (open ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION)) {
                    return true;
                }
                setClaw(open);
                started = true;
                time.reset();
            }

            return time.seconds() > CLAW_MOVE_TIME;
        }
    }

    public Action getClawAction(boolean open) {
        return new ClawAction(open);
    }

    private class MoveLiftToPosition implements Action {
        private boolean started = false;
        private final LiftPosition position;

        public MoveLiftToPosition(LiftPosition position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!started) {
                setPosition(position);
                started = true;
            }
            periodic();
            return !getState().isFinished();
        }
    }

    public Action moveLiftToPosition(LiftPosition position) {
        return new MoveLiftToPosition(position);
    }

}
