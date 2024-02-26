package org.firstinspires.ftc.teamcode.opmode.auto.oldautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class BlueAudiencePurplePixelAuto extends BlueBoardPurplePixelAuto {
    public BlueAudiencePurplePixelAuto() {
        isRed = false;
        closeRightTurn = true;
    }
}
