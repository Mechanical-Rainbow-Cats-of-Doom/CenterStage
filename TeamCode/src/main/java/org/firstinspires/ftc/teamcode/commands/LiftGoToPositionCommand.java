package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.NewLift;


public class LiftGoToPositionCommand extends CommandBase {
    public NewLift lift;
    public NewLift.LiftPosition position;

    public LiftGoToPositionCommand(NewLift lift, NewLift.LiftPosition position) {
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
