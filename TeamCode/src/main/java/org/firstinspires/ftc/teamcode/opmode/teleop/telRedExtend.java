package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@TeleOp(name="Red Extended")
public class telRedExtend extends telopAltHeading {

    @Override
    public void init() {
        super.init();
        this.robot.alliance = Robot.Alliance.RED;

    }
}
