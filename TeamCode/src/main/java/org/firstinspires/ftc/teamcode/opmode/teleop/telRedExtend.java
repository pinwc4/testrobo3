package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Red Extended")
public class telRedExtend extends telopAltHeading {

    @Override
    public void init() {
        super.init();
        this.alliance = Alliance.RED;
    }
}
