package org.firstinspires.ftc.teamcode.opmode.auto.oldautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmode.auto.oldautos.BlueBoardPurplePixelAuto;

@Autonomous
@Disabled
public class RedAudiencePurplePixelAuto extends BlueBoardPurplePixelAuto {
    public RedAudiencePurplePixelAuto() {
        isRed = true;
        closeRightTurn = false;
    }
}
