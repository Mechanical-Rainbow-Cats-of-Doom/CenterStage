package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.NewIntake;
import org.firstinspires.ftc.teamcode.tool.NewLift;

public class SetIntakeCommand extends CommandBase {
    public final NewIntake intake;
    public NewIntake.State state;
    public NewLift lift;

    public SetIntakeCommand(NewIntake intake, NewIntake.State state, NewLift lift) {
        this.intake = intake;
        this.state = state;
        this.lift = lift;
    }

    public SetIntakeCommand(NewIntake intake, NewIntake.State state) {
        this(intake, state, null);
    }

    @Override
    public void execute() {
        if(lift == null || lift.getState() == NewLift.State.AT_POSITION) {
            intake.setState(state);
        }
    }

    @Override
    public boolean isFinished() {
        return state == NewIntake.State.OFF;
    }
}
