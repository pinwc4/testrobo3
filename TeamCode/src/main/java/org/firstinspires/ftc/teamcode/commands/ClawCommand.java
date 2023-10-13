package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ClawCommand extends CommandBase {
    private final Claw clawSubsystem;

    public ClawCommand(Robot robot) {
        clawSubsystem = robot.clawSubsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
