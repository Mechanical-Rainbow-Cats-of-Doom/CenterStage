package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.tool.Lift;

import java.util.Set;

public class LiftSafeCommand implements Command {
    public LiftSafeCommand(Intake intake, Lift lift) {
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return null;
    }
}
