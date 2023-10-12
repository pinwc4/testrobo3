package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Claw extends SubsystemBase {

    enum State {
        OPEN,
        CLOSED
    }

    private Robot robotBase;
    public double position;

    public State clawState;


    public Claw(Robot robot) {
        robotBase = robot;
    }

    public void open() {
        robotBase.claw.setPosition(0.5);
        position = 0.5;
        clawState = State.OPEN;
    }

    public void close() {
        robotBase.claw.setPosition(0);
        position = 0;
        clawState = State.CLOSED;
    }

    public void toggle() {
        if (clawState == State.CLOSED) {
            open();
        } else {
            close();
        }
    }

    public double getPosition() {
        return position;
    }

    public State getState () {
        return clawState;
    }

}
