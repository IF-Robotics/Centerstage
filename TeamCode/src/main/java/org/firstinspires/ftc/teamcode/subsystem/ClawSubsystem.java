package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class ClawSubsystem extends SubsystemBase {
    private ServoEx left, right;
    public double open = 1, close = 0; //TODO: actually make these numbers real, might need to make one for L and R

    public ClawSubsystem(ServoEx left, ServoEx right) {
        this.left = left;
        this.right = right;
    }

    public void setLeft(double position) {
        left.setPosition(position);
    }
    public void setRight(double position) {
        right.setPosition(position);
    }

    @Override
    public void periodic() {

    }
}
