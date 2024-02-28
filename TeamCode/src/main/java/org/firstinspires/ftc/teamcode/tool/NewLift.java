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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.LiftGoToPositionCommand;

import java.util.Arrays;

@Config
public class NewLift extends SubsystemBase {
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
        boolean safeArmPitch();
        Time getLiftMoveTime();
        Time getLiftMoveTwoTime();
        Time getArmPitchTime();
        Time getArmRollTime();
        Time getCarriageRollTime();
        Time getArmLengthTime();
        Time getClawTime();
        Time getRetractArmTime();
        Time getSafeArmPitchTime();
        Time getLiftBlockingTime();
        LiftPosition getNextLiftPosition();

        /**
         * Represents the point in time where something will happen. Very late requires a button
         * press before it starts.
         */
        enum Time {
            VV_EARLY(1000), VERY_EARLY(2000), EARLY(3000),
            NORMAL(4000), LATE(5000), VERY_LATE_BUTTON(6000);

            private final int index;

            Time(int index) {
                this.index = index;
            }
        }

        enum Default implements LiftPosition {
            DOWN(0, 140, false, 0.43, 0.535, 0.85, false, true, true,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.VERY_EARLY, Time.VERY_EARLY, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, Time.VV_EARLY, null),
            T_LEFT_LOW(1120, 140, true, 0.8, 0.76, 0.35, true, true, false,
                    Time.NORMAL, Time.EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_LATE_BUTTON, Time.VERY_EARLY, null, null, Default.DOWN),
            T_LEFT_MEDIUM(1960, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_LEFT_LOW.armRoll, T_LEFT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_LEFT_HIGH(2870, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_LEFT_LOW.armRoll, T_LEFT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_LEFT_VHIGH(3900, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.76, 0.6, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),

            T_MIDDLE_LOW(560, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_MIDDLE_MEDIUM(1400, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_MIDDLE_HIGH(2310, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_MIDDLE_VHIGH(3700, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),

            T_RIGHT_LOW(1120, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.14, 0.33, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_RIGHT_MEDIUM(1960, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_RIGHT_LOW.armRoll, T_RIGHT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_RIGHT_HIGH(2870, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_RIGHT_LOW.armRoll, T_RIGHT_LOW.carriageRoll, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),
            T_RIGHT_VHIGH(2870, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.14, 0.47, T_LEFT_LOW.armLength, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime, T_LEFT_LOW.nextLiftPosition),


            T_MIDDLE_LOW_LTR(560, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_RIGHT_LOW),
            T_MIDDLE_MEDIUM_LTR(1400, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_RIGHT_MEDIUM),
            T_MIDDLE_HIGH_LTR(2310, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_RIGHT_HIGH),
            T_MIDDLE_VHIGH_LTR(3700, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_RIGHT_VHIGH),

            T_MIDDLE_LOW_RTL(560, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_LEFT_LOW),
            T_MIDDLE_MEDIUM_RTL(1400, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_LEFT_MEDIUM),
            T_MIDDLE_HIGH_RTL(2310, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_LEFT_HIGH),
            T_MIDDLE_VHIGH_RTL(3700, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, 0.43, 0.535, 0.75, T_LEFT_LOW.clawOpen, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime,
                    T_LEFT_VHIGH),




            // NO WORK
            A_VLOW_MIDDLEDUMP_RIGHT_LEAN(0, 140, true, 0.7, 0.535, 1, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            // NO WORK
            A_VLOW_MIDDLEDUMP_LEFT_LEAN(0, 140, true, 0.7, 0.535, 1, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_MIDDLE_VLOW_LEFTDUMP(280, 140, true, 0.25, 0.3, 1, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_MIDDLE_VLOW_RIGHTDUMP(140, 140, true, 0.5, 0.75, 1, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),

            A_LEFT_VLOW_LEFTDUMP(800, 140, true, 0.8, 0.72, 0.35, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_LEFT_VLOW_MIDDLEDUMP(500, 140, true, 0.8, 0.98, 0.35, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_LEFT_VLOW_RIGHTDUMP(700, 140, true, 0.8, 1, 0.35, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_LEFT_LOW(T_LEFT_LOW.liftTicks, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_LEFT_LOW.armRoll, T_LEFT_LOW.carriageRoll, T_LEFT_LOW.armLength, false, T_LEFT_LOW.retractArm, T_LEFT_LOW.safeArmPitch,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, null, T_LEFT_LOW.clawTime, T_LEFT_LOW.retractArmTime, T_LEFT_LOW.safeArmPitchTime, T_LEFT_LOW.liftBlockingTime),

            A_RIGHT_VLOW_RIGHTDUMP(400, 140, true, 0.06, 0.45, 0.34, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_RIGHT_VLOW_MIDDLEDUMP(420, 140, true, 0.06, 0.15, 0.34, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_RIGHT_VLOW_LEFTDUMP(750, 140, true, 0.06, 0.1, 0.34, false, true, false,
                    Time.NORMAL, Time.VERY_EARLY, Time.EARLY, Time.NORMAL, Time.NORMAL, Time.NORMAL, Time.VERY_EARLY, Time.VERY_EARLY, null, null),
            A_RIGHT_LOW(T_LEFT_LOW.liftTicks, T_LEFT_LOW.liftTicksTwo, T_LEFT_LOW.armOut, T_RIGHT_LOW.armRoll, T_RIGHT_LOW.carriageRoll, T_RIGHT_LOW.armLength, false, T_RIGHT_LOW.retractArm, false,
                    T_LEFT_LOW.liftMoveTime, T_LEFT_LOW.liftMoveTwoTime, T_LEFT_LOW.armPitchTime, T_LEFT_LOW.armRollTime, T_LEFT_LOW.carriageRollTime, T_LEFT_LOW.armLengthTime, null, T_LEFT_LOW.retractArmTime, null, null);

            private final int liftTicks;
            private final int liftTicksTwo;
            private final boolean armOut;
            private final double armRoll;
            private final double carriageRoll;
            private final double armLength;
            private final boolean retractArm;
            private final boolean safeArmPitch;
            private final Time liftMoveTime;
            private final Time liftMoveTwoTime;
            private final Time armRollTime;
            private final Time carriageRollTime;
            private final Time armPitchTime;
            private final Time armLengthTime;
            private final Time clawTime;
            private final Time retractArmTime;
            private final Time safeArmPitchTime;
            private final Time liftBlockingTime;
            private final boolean clawOpen;
            private final LiftPosition nextLiftPosition;

            Default(int liftTicks, int liftTicksTwo, boolean armOut, double armRoll, double carriageRoll,
                    double armLength, boolean clawOpen, boolean retractArm, boolean safeArmPitch,
                    Time liftMoveTime, Time liftMoveTwoTime, Time armPitchTime, Time armRollTime, Time carriageRollTime,
                    Time armLengthTime, Time clawTime, Time retractArmTime, Time safeArmPitchTime, Time liftBlockingTime,
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
                this.armPitchTime = armPitchTime;
                this.armLengthTime = armLengthTime;
                this.clawTime = clawTime;
                this.nextLiftPosition = nextLiftPosition;
                this.retractArm = retractArm;
                this.retractArmTime = retractArmTime;
                this.safeArmPitch = safeArmPitch;
                this.safeArmPitchTime = safeArmPitchTime;
                this.liftBlockingTime = liftBlockingTime;
            }

            Default(int liftTicks, int liftTicksTwo, boolean armOut, double armRoll, double carriageRoll,
                    double armLength, boolean clawOpen, boolean retractArm, boolean safeArmPitch,
                    Time liftMoveTime, Time liftMoveTwoTime, Time armPitchTime, Time armRollTime, Time carriageRollTime,
                    Time armLengthTime, Time clawTime, Time retractArmTime, Time safeArmPitchTime, Time liftBlockingTime) {
                this(liftTicks, liftTicksTwo, armOut, armRoll, carriageRoll, armLength, clawOpen, retractArm,
                        safeArmPitch,
                        liftMoveTime, liftMoveTwoTime, armPitchTime, armRollTime, carriageRollTime,
                        armLengthTime, clawTime, retractArmTime, safeArmPitchTime, liftBlockingTime, null);
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
            public Time getArmPitchTime() {
                return armPitchTime;
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
            public boolean safeArmPitch() {
                return safeArmPitch;
            }

            @Override
            public Time getRetractArmTime() {
                return retractArmTime;
            }

            @Override
            public Time getSafeArmPitchTime() {
                return safeArmPitchTime;
            }

            @Override
            public Time getLiftBlockingTime() {
                return liftBlockingTime;
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
            public Time getArmPitchTime() {
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
            public boolean safeArmPitch() {
                return false;
            }

            @Override
            public Time getRetractArmTime() {
                return retractArmTime;
            }

            @Override
            public Time getSafeArmPitchTime() {
                return null;
            }

            @Override
            public Time getLiftBlockingTime() {
                return null;
            }

            @Override
            public LiftPosition getNextLiftPosition() {
                return nextLiftPosition;
            }
        }
    }
    public enum State {
        STARTING_MOVE(false),
        VV_EARLY_MOVE(false, LiftPosition.Time.VV_EARLY),
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

        public boolean isAtOrAfter(LiftPosition.Time time) {
            if(this.time == null) return true;
            return time.index <= this.time.index;
        }
    }

    public static double LIFT_POWER = 1;
    public static double kP = 1.2e-2;
    public static double POSITION_TOLERANCE = 15;
    public static int POSITION_LIMIT = 4200;
    public static double CLAW_OPEN_POSITION = 0;
    public static double CLAW_CLOSED_POSITION = 0.2;
    public static double ARM_IN_PITCH = 0.32;
    public static double ARM_SAFE_PITCH = 0.40;
    public static double ARM_OUT_PITCH = 0.44;
    public static double ARM_PITCH_MOVE_TIME = 1.33;
    public static double ARM_PITCH_SAFE_TIME = 0.2;
    public static double CLAW_MOVE_TIME = 0.2;
    public static double CLAW_DROP_TIME = 1;
    public static double ARM_ROLL_TIME_P = 0.79;
    public static double CARRIAGE_ROLL_TIME_P = 0.6;
    public static double ARM_LENGTH_TIME_P = 1.3;
    public static double FIRST_SERVO_MIDDLE = 0.5;
    public static double SECOND_SERVO_MIDDLE = 0.5;
    public static boolean SECOND_SERVO_INVERTED = true;
    public static double ARM_RETRACTED_POSITION = 1;
    public static double LIFT_TICK_MULTIPLIER = Motor.GoBILDA.RPM_117.getCPR()/Motor.GoBILDA.RPM_223.getCPR();

    public static double MIN_CARRIAGE_ANGLE = 0;
    public static double MAX_CARRIAGE_ANGLE = 400;
    public static double MIN_ARM_ANGLE = 0;
    public static double MAX_ARM_ANGLE = 400;
    public static double ARM_ANGLE_OFFSET = -8;
    public static double MIN_CLIPPED_ARM_ANGLE = 0;
    public static double MAX_CLIPPED_ARM_ANGLE = 320;
    public static double MIN_CLIPPED_CARRIAGE_ANGLE = 100;
    public static double MAX_CLIPPED_CARRIAGE_ANGLE = 300;
    public static double CARRIAGE_ANGLE_OFFSET = 40;

    public static double FLICK_STICK_DEADZONE = 0.8;
    public static boolean FLICK_STICK_ARM = true;
    public static boolean FLICK_STICK_CARRIAGE = true;
    public static double FORCE_UPDATE_RATE = 0.1;

    public int direction = 1;
    public int height = 0;

    private final MotorGroup liftMotor;
    private final Motor liftLeader;
    private final SimpleServo armPitchPrimaryRight;
    private final ServoEx armPitchSecondaryLeft;
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
        public boolean safeArmPitch() {
            return false;
        }

        @Override
        public Time getLiftMoveTime() {
            return null;
        }

        @Override
        public Time getLiftMoveTwoTime() {
            return null;
        }

        @Override
        public Time getArmPitchTime() {
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
        public Time getSafeArmPitchTime() {
            return null;
        }

        @Override
        public Time getLiftBlockingTime() {
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
    private final ElapsedTime currentForceUpdateTime = new ElapsedTime();
    private final ElapsedTime time = new ElapsedTime();
    private boolean initial = true;

    private double lastKP = kP;
    private double lastPositionTolerance = POSITION_TOLERANCE;
    private double lastMinCarriageAngle = MIN_CARRIAGE_ANGLE;
    private double lastMaxCarriageAngle = MAX_CARRIAGE_ANGLE;
    private double lastMinArmAngle = MIN_ARM_ANGLE;
    private double lastMaxArmAngle = MAX_ARM_ANGLE;


    private final boolean isTeleOp;
    private final Telemetry telemetry;

    @SuppressWarnings("ConstantConditions")
    public NewLift(HardwareMap hardwareMap, GamepadEx toolGamepad, Telemetry telemetry, boolean debug, boolean teleOp) {
        final Motor.GoBILDA motorType = Motor.GoBILDA.RPM_117;
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

        armPitchSecondaryLeft = new SimpleServo(hardwareMap, "armYawLeft", 0, 1);
        armPitchPrimaryRight = new SimpleServo(hardwareMap, "armYawRight", 0, 1);
        armRollServo = new SimpleServo(hardwareMap, "armRoll", MIN_ARM_ANGLE, MAX_ARM_ANGLE);
        armLengthServo = new SimpleServo(hardwareMap, "armLength", 0, 1);
        carriageRollServo = new SimpleServo(hardwareMap, "carriageRoll", MIN_CARRIAGE_ANGLE, MAX_CARRIAGE_ANGLE);
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
        this.telemetry = telemetry;
    }

    public NewLift(HardwareMap hardwareMap, GamepadEx toolGamepad, boolean debug) {
        this(hardwareMap, toolGamepad, null, debug, true);
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
        if(lastMinArmAngle != MIN_ARM_ANGLE || lastMaxArmAngle != MAX_ARM_ANGLE) {
            armRollServo.setRange(MIN_ARM_ANGLE, MAX_ARM_ANGLE);
            lastMinArmAngle = MIN_ARM_ANGLE;
            lastMaxArmAngle = MAX_ARM_ANGLE;
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
            if(Math.pow(Math.abs(toolGamepad.getRightY()), 2) + Math.pow(Math.abs(toolGamepad.getRightX()), 2) > FLICK_STICK_DEADZONE*FLICK_STICK_DEADZONE) {
                double angle = Math.toDegrees(Math.atan2(toolGamepad.getRightX(), toolGamepad.getRightY()));
                if(telemetry != null)
                    telemetry.addData("Flick Stick Angle", angle);
                if(FLICK_STICK_CARRIAGE)
                    carriageRollServo.turnToAngle(Range.clip(unsignedMod(-angle + CARRIAGE_ANGLE_OFFSET, 360), MIN_CLIPPED_CARRIAGE_ANGLE, MAX_CLIPPED_CARRIAGE_ANGLE));
                if(FLICK_STICK_ARM)
                    armRollServo.turnToAngle(Range.clip(unsignedMod(-angle + ARM_ANGLE_OFFSET, 360), MIN_CLIPPED_ARM_ANGLE, MAX_CLIPPED_ARM_ANGLE));
            }
            return;
        }
        if(runMode != Motor.RunMode.PositionControl) {
            liftMotor.setRunMode(Motor.RunMode.PositionControl);
            runMode = Motor.RunMode.PositionControl;
        }

        liftMotor.set(1);
        switch(state) {
            case STARTING_MOVE:
                state = State.VV_EARLY_MOVE;
                waitingForLiftMove = false;
                maxMoveTime = 0;
                currentForceUpdateTime.reset();
                if(previousLiftPosition.isArmOut() != position.isArmOut()) {
                    prepareMove();
                    time.reset();
                }
            case VV_EARLY_MOVE:
                if(previousLiftPosition.isArmOut() != position.isArmOut() || initial) {
                    doForceMove();
                    if (waitingForLiftMove && !liftMotor.atTargetPosition()) {
                        break;
                    }
                    if (maxMoveTime > time.seconds()) {
                        break;
                    }
                    waitingForLiftMove = false;
                    maxMoveTime = 0;
                    state = State.VERY_EARLY_MOVE;
                    prepareMove();
                    time.reset();
                }
            case VERY_EARLY_MOVE:
                if(previousLiftPosition.isArmOut() != position.isArmOut() || initial) {
                    doForceMove();
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
                    doForceMove();
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
                doForceMove();
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
                doForceMove();
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
                    if (toolGamepad != null && !toolGamepad.getButton(GamepadKeys.Button.Y)) {
                        break;
                    }
                    state = State.VERY_LATE_BUTTON_MOVE;
                    prepareMove();
                    time.reset();
                }
            case VERY_LATE_BUTTON_MOVE:
                doForceMove();
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
                doForceMove();
                LiftPosition next = position.getNextLiftPosition();
                if(next != null && (toolGamepad == null || toolGamepad.getButton(GamepadKeys.Button.Y))) {
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

    public boolean isArmDown() {
        return armPitchPrimaryRight.getPosition() == ARM_IN_PITCH;
    }

    public void toggleArmPosition() {
        if(!automatic) {
            double requiredPosition = (armPitchPrimaryRight.getPosition() == ARM_IN_PITCH) ? ARM_OUT_PITCH : ARM_IN_PITCH;

            armPitchPrimaryRight.setPosition(requiredPosition);
            armPitchSecondaryLeft.setPosition((requiredPosition-FIRST_SERVO_MIDDLE)*(SECOND_SERVO_INVERTED ? -1 : 1)+SECOND_SERVO_MIDDLE);
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
                position.getArmLengthTime() == time || position.getArmPitchTime() == time ||
                position.getArmRollTime() == time || position.getCarriageRollTime() == time ||
                position.getRetractArmTime() == time || position.getLiftMoveTwoTime() == time ||
                position.getSafeArmPitchTime() == time || position.getLiftBlockingTime() == time;
    }

    private void prepareMove() {
        if(state.currentTime(position.getLiftMoveTime())) {
            double ticks = Math.max(0, Math.min(position.getLiftTicks()*LIFT_TICK_MULTIPLIER, POSITION_LIMIT*LIFT_TICK_MULTIPLIER));
            liftMotor.setTargetPosition((int)Math.round(ticks));
            if(position.getLiftBlockingTime() == null) {
                waitingForLiftMove = true;
            }
        } else if(state.currentTime(position.getLiftMoveTwoTime())) {
            double ticks = Math.max(0, Math.min(position.getLiftTicksTwo()*LIFT_TICK_MULTIPLIER, POSITION_LIMIT*LIFT_TICK_MULTIPLIER));
            liftMotor.setTargetPosition((int)Math.round(ticks));
            waitingForLiftMove = true;
        }
        if(state.currentTime(position.getLiftBlockingTime())) {
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
        if(state.currentTime(position.getArmPitchTime())) {
            double requiredPosition = position.isArmOut() ? ARM_OUT_PITCH : ARM_IN_PITCH;
            if(armPitchPrimaryRight.getPosition() != requiredPosition || initial) {
                maxMoveTime = Math.max(maxMoveTime, ARM_PITCH_MOVE_TIME);
                armPitchPrimaryRight.setPosition(requiredPosition);
                armPitchSecondaryLeft.setPosition((requiredPosition-FIRST_SERVO_MIDDLE)*(SECOND_SERVO_INVERTED ? -1 : 1)+SECOND_SERVO_MIDDLE);
            }
        } else if(position.safeArmPitch() && state.currentTime(position.getSafeArmPitchTime())) {
            double requiredPosition = ARM_SAFE_PITCH;
            if(armPitchPrimaryRight.getPosition() != requiredPosition || initial) {
                maxMoveTime = Math.max(maxMoveTime, ARM_PITCH_SAFE_TIME);
                armPitchPrimaryRight.setPosition(requiredPosition);
                armPitchSecondaryLeft.setPosition((requiredPosition-FIRST_SERVO_MIDDLE)*(SECOND_SERVO_INVERTED ? -1 : 1)+SECOND_SERVO_MIDDLE);
            }
        }
        if(state.currentTime(position.getClawTime())) {
            double requiredPosition = position.isClawOpen() ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
            if(carriageClawServo.getPosition() != requiredPosition || initial) {
                boolean isDropping = armPitchPrimaryRight.getPosition() == ARM_OUT_PITCH && position.isClawOpen();
                maxMoveTime = Math.max(maxMoveTime, (isDropping) ? CLAW_DROP_TIME : CLAW_MOVE_TIME);
                carriageClawServo.setPosition(requiredPosition);
            }
        }
    }

    private void doForceMove() {
        if(currentForceUpdateTime.seconds() < FORCE_UPDATE_RATE) return;

        currentForceUpdateTime.reset();
        if(state.isAtOrAfter(position.getArmRollTime())) {
            armRollServo.setPosition(position.getArmRoll());
        }
        if(state.isAtOrAfter(position.getCarriageRollTime())) {
            carriageRollServo.setPosition(position.getCarriageRoll());
        }
        if(state.isAtOrAfter(position.getArmLengthTime())) {
            armLengthServo.setPosition(position.getArmLength());
        }
        if(state.isAtOrAfter(position.getArmPitchTime())) {
            double requiredPosition = position.isArmOut() ? ARM_OUT_PITCH : ARM_IN_PITCH;
            armPitchPrimaryRight.setPosition(requiredPosition);
            armPitchSecondaryLeft.setPosition((requiredPosition-FIRST_SERVO_MIDDLE)*(SECOND_SERVO_INVERTED ? -1 : 1)+SECOND_SERVO_MIDDLE);
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
            int diff = Math.abs(o - direction);

            if(height) {
                NewLift.this.height = Math.max(0, Math.min(NewLift.this.height + o, 3));
            } else {
                NewLift.this.direction = o;
            }

            chooseNewLiftPosition(diff == 2 && !height);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private void chooseNewLiftPosition(boolean opposingSides) {
        direction = direction % 3;
        height = height % 4;
        setPosition(selectPosition(opposingSides, direction, height));
    }

    private static LiftPosition selectPosition(boolean opposingSides, int direction, int height) {
        switch(direction) {
            case 0:
                switch(height) {
                    case 0:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_LOW_RTL : LiftPosition.Default.T_LEFT_LOW;
                    case 1:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_MEDIUM_RTL : LiftPosition.Default.T_LEFT_MEDIUM;
                    case 2:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_HIGH_RTL: LiftPosition.Default.T_LEFT_HIGH;
                    case 3:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_VHIGH_RTL : LiftPosition.Default.T_LEFT_VHIGH;
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
                    case 3:
                        return LiftPosition.Default.T_MIDDLE_VHIGH;
                    default:
                        return LiftPosition.Default.DOWN;
                }
            case 2:
                switch(height) {
                    case 0:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_LOW_LTR : LiftPosition.Default.T_RIGHT_LOW;
                    case 1:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_MEDIUM_LTR : LiftPosition.Default.T_RIGHT_MEDIUM;
                    case 2:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_HIGH_LTR : LiftPosition.Default.T_RIGHT_HIGH;
                    case 3:
                        return opposingSides ? LiftPosition.Default.T_MIDDLE_VHIGH_LTR : LiftPosition.Default.T_RIGHT_VHIGH;
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
        private final ElapsedTime time = new ElapsedTime();
        private final boolean open;
        private boolean started = false;

        public ClawAction(boolean open) {
            this.open = open;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!started) {
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

    private double unsignedMod(double toMod, double modBy) {
        double out = toMod % modBy;
        if(out < 0) {
            out = modBy + out;
        }
        return out;
    }
}
