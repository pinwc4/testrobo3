package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class RobotBase {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public Arm armSubsystem;
    public Claw clawSubsystem;

    public RobotBase(HardwareMap hwMap) {
        frontLeftMotor = hwMap.dcMotor.get("leftFront");
        backLeftMotor = hwMap.dcMotor.get("leftRear");
        frontRightMotor = hwMap.dcMotor.get("rightFront");
        backRightMotor = hwMap.dcMotor.get("rightRear");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        armSubsystem = new Arm(hwMap.servo.get("arm"));
        clawSubsystem = new Claw(hwMap.servo.get("claw"));
    }

}
