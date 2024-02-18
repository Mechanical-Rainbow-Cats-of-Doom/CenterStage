package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.old.OldLift;


public class LiftGoToPositionCommand extends CommandBase {
    public OldLift lift;
    public OldLift.LiftPosition position;

    public LiftGoToPositionCommand(OldLift lift, OldLift.LiftPosition position) {
        this.lift = lift;
        this.position = position;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        if(lift.getPosition() != position) {
            lift.setPosition(position);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
