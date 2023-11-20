package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {
    public enum LiftPosition {
        DOWN(0),
        ONE(0),
        TWO(0),
        THREE(0);

        public final int ticks;

        LiftPosition(int ticks) {
            this.ticks = ticks;
        }
    }
    private final Motor motor;

    public Lift(HardwareMap hardwareMap) {
        this.motor = new MotorGroup(new Motor(hardwareMap, "lift1", Motor.GoBILDA.RPM_435));
    }

    private LiftPosition pos;

    @Override
    public void periodic() {
    }

}
