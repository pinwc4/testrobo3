package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ClawCloseCommand extends CommandBase {
    private final Claw clawSubsystem;

    public ClawCloseCommand(Claw clawSub) {
        clawSubsystem = clawSub;
        //addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return clawSubsystem.isClosed();
    }
}
