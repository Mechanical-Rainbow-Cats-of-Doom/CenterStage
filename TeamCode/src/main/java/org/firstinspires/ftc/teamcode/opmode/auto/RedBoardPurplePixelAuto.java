package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedBoardPurplePixelAuto extends BlueBoardPurplePixelAuto {
    public RedBoardPurplePixelAuto() {
        isRed = true;
        closeRightTurn = true;
    }
}
