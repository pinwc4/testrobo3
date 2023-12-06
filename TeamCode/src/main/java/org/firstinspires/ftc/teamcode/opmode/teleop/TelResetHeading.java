package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DataStorage;

@TeleOp(name="Reset Heading")
public class TelResetHeading extends telopAltHeading {

    @Override
    public void init() {
        super.init();
        this.robot.alliance = Robot.Alliance.RED;
        DataStorage.finalAutoHeading = 0;
    }
}