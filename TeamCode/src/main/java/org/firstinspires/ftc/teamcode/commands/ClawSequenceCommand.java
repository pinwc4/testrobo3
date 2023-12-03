package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ClawSequenceCommand extends SequentialCommandGroup {
    public ClawSequenceCommand(Claw clawSubCon) {
        new InstantCommand(() -> clawSubCon.close());
        new WaitUntilCommand(()->clawSubCon.isClosed());
        new WaitCommand(500);
        new InstantCommand(()->clawSubCon.open());
    }
}
