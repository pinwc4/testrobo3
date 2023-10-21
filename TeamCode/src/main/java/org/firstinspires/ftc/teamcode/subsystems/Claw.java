package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Claw extends SubsystemBase {

    final double openPosition = 0.5;
    final double closedPosition = 0;

    enum State {
        OPEN,
        CLOSED
    }

    enum Period {
        NULL,
        RUNNING
    }

    public double position;

    public State clawState;
    public Period period;
    private Servo clawServo;


    public Claw(Servo t_servo) {
        clawServo = t_servo;
        close();
    }

    public void open() {
        clawServo.setPosition(openPosition);
        position = openPosition;
        clawState = State.OPEN;
    }

    public void close() {
        clawServo.setPosition(closedPosition);
        position = closedPosition;
        clawState = State.CLOSED;
    }

    public void toggle() {
        if (clawState == State.CLOSED) {
            open();
        } else {
            close();
        }
    }

    public void periodic() {
        period = Period.RUNNING;
    }

    public double getPosition() {
        return position;
    }

    public State getState () {
        return clawState;
    }

}
