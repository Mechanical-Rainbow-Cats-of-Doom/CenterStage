package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAudiencePlacePixelAuto extends BlueAudiencePlacePixelAuto {
    public RedAudiencePlacePixelAuto() {
        isRed = true;
        closeRightTurn = true;
    }
}
