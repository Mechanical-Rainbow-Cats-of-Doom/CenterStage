package org.firstinspires.ftc.teamcode.opmode.auto.oldautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auto.oldautos.BlueBoardPurplePixelAuto;

@Autonomous
public class RedBoardPurplePixelAuto extends BlueBoardPurplePixelAuto {
    public RedBoardPurplePixelAuto() {
        isRed = true;
        closeRightTurn = true;
    }
}
