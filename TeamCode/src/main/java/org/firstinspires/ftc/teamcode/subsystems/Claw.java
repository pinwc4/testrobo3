package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo srvClaw;
    private boolean bolClawOpen;
    private double dblOpenPosition = 0;
    private double dblClosedPosition = 1;

    public Claw(Servo m_srvClaw) {
        srvClaw = m_srvClaw;
        openClaw();
    }

    public void openClaw() {
        bolClawOpen = true;
        srvClaw.setPosition(dblOpenPosition);
    }

    public void closeClaw() {
        bolClawOpen = false;
        srvClaw.setPosition(dblClosedPosition);
    }

    public boolean isClawOpen() {
        if (bolClawOpen) {
            return true;
        } else {
            return false;
        }
    }

    public void toggleClaw() {
        if (isClawOpen()) {
            closeClaw();
        } else {
            openClaw();
        }
    }
}
