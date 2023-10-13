package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Claw extends SubsystemBase {

    enum State {
        OPEN,
        CLOSED
    }

    enum Period {
        NULL,
        RUNNING
    }

    //private Robot robotBase;
    public double position;

    public State clawState;
    public Period period;
    private Servo clawServo;


    public Claw(Servo t_servo) {
        clawServo = t_servo;
        close();
    }

    public void open() {
        clawServo.setPosition(0.5);
        position = 0.5;
        clawState = State.OPEN;
    }

    public void close() {
        clawServo.setPosition(0);
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
