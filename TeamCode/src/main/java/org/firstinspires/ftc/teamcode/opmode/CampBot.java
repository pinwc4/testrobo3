package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.RobotBase;

@TeleOp(name = ("Camp Bot"))
public class CampBot extends OpMode {
    private RobotBase robotBase;

    @Override
    public void init() {
        robotBase = new RobotBase(hardwareMap);
    }

    @Override
    public void loop() {

    }

}
