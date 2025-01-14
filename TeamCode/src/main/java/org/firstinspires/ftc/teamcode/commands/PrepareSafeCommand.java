//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//
//import org.firstinspires.ftc.teamcode.tool.OldIntake;
//import org.firstinspires.ftc.teamcode.tool.old.OldLift;
//
//public class PrepareSafeCommand extends CommandBase {
//    public final LiftSafeCommand safeCommand;
//    public final OldIntake intake;
//    public final OldLift lift;
//
//    public PrepareSafeCommand(OldIntake intake, OldLift lift) {
//        this.intake = intake;
//        this.lift = lift;
//        safeCommand = new LiftSafeCommand(intake, lift);
//    }
//
//
//    @Override
//    public void execute() {
//        if(intake.getOn() && intake.getState() && !CommandScheduler.getInstance().isScheduled(safeCommand.postSafe)) {
//            CommandScheduler.getInstance().schedule(false, safeCommand);
//        }
//    }
//}
