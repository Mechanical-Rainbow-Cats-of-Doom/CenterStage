package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.Lift;


public class LiftGoToPositionCommand extends CommandBase {
    public Lift lift;
    public Lift.LiftPosition position;

    public LiftGoToPositionCommand(Lift lift, Lift.LiftPosition position) {
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
