package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public enum ArmPosition {
        HOME(0.11556),
        GROUND(0),
        LOW(0),
        MEDIUM(0),
        HIGH(0);
        public final double value;
        ArmPosition(double m_position){
            this.value = m_position;
        }
    }

    Servo srvArm;
    ArmPosition enuArmPosition;

    public Arm(Servo m_srvArm) {
        srvArm = m_srvArm;
        goToPosition(ArmPosition.HOME);
    }

    public void goToPosition(ArmPosition m_enumArmPosition) {
        enuArmPosition = m_enumArmPosition;
        srvArm.setPosition(enuArmPosition.value);
    }

    public ArmPosition getPosition() {
        return enuArmPosition;
    }

}
