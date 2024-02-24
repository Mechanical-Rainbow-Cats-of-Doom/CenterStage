package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.NewIntake;

public class SetIntakeCommand extends CommandBase {
    public final NewIntake intake;
    public NewIntake.State state;

    public SetIntakeCommand(NewIntake intake, NewIntake.State state) {
        this.intake = intake;
        this.state = state;
    }

    @Override
    public void execute() {
        intake.setState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
