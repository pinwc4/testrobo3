package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.base.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp(name = ("Camp Bot"))
public class CampBot extends OpMode {
    private RobotBase robotBase;
    private Gamepad currentGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad1;
    private Gamepad previousGamepad2;

    @Override
    public void init() {
        robotBase = new RobotBase(hardwareMap);
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            robotBase.clawSubsystem.toggleClaw();
        }

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            robotBase.armSubsystem.goToPosition(Arm.ArmPosition.HIGH);
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            robotBase.armSubsystem.goToPosition(Arm.ArmPosition.MEDIUM);
        }

        if (currentGamepad1.x && !previousGamepad1.x) {
            robotBase.armSubsystem.goToPosition(Arm.ArmPosition.LOW);
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            robotBase.armSubsystem.goToPosition(Arm.ArmPosition.GROUND);
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            robotBase.armSubsystem.goToPosition(Arm.ArmPosition.HOME);
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robotBase.frontLeftMotor.setPower(frontLeftPower);
        robotBase.backLeftMotor.setPower(backLeftPower);
        robotBase.frontRightMotor.setPower(frontRightPower);
        robotBase.backRightMotor.setPower(backRightPower);
    }

}
