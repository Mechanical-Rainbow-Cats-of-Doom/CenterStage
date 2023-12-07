package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.ToggleableMotor;

public class TogglePowerToggleableMotor extends CommandBase {
    private final ToggleableMotor m;

    public TogglePowerToggleableMotor(ToggleableMotor m) {
        this.m = m;
    }

    @Override
    public void execute() {
        m.toggleOn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
