//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.commands.ControllerDriveCommand;
//import org.firstinspires.ftc.teamcode.commands.PrepareSafeCommand;
//import org.firstinspires.ftc.teamcode.commands.SetIntakeCommands;
//import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveSubsystem;
//import org.firstinspires.ftc.teamcode.tool.DroneLauncher;
//import org.firstinspires.ftc.teamcode.tool.Intake;
//import org.firstinspires.ftc.teamcode.tool.Lift;
//
//@TeleOp(name = "🐟 squared eenputs 🐟")
//public class SqInputsMecanumOpMode extends MecanumOpMode {
//    public SqInputsMecanumOpMode() {
//        squareInputs = true;
//    }
//}
