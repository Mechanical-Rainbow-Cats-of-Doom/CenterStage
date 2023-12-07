package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.ToggleableMotor;

public class ToggleDirectionToggleableMotor extends CommandBase {
    private final ToggleableMotor m;

    public ToggleDirectionToggleableMotor(ToggleableMotor m) {
        this.m = m;
    }

    @Override
    public void execute() {
        m.toggleState();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
