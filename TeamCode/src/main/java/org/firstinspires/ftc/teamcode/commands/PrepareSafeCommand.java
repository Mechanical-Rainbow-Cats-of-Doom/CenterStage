package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.tool.Lift;

public class PrepareSafeCommand extends CommandBase {
    public final Intake intake;
    public final Lift lift;

    public PrepareSafeCommand(Intake intake, Lift lift) {
        this.intake = intake;
        this.lift = lift;
    }


    @Override
    public void execute() {
        if(intake.getOn() && intake.getState()) {
            CommandScheduler.getInstance().schedule(false, new LiftSafeCommand(intake, lift));
        }
    }
}
