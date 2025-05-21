package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotBase {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public RobotBase(HardwareMap hwMap) {
        frontLeftMotor = hwMap.dcMotor.get("leftFront");
        backLeftMotor = hwMap.dcMotor.get("leftRear");
        frontRightMotor = hwMap.dcMotor.get("rightFront");
        backRightMotor = hwMap.dcMotor.get("rightRear");
    }

}
