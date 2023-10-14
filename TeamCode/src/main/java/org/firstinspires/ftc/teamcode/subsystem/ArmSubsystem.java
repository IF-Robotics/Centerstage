package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmSubsystem extends SubsystemBase {
    private DcMotorEx slide1, slide2;
    private ServoEx arm1, arm2, wrist;

    private double slidePower;

    public ArmSubsystem(DcMotorEx slide1, DcMotorEx slide2, ServoEx arm1, ServoEx arm2, ServoEx wrist) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.wrist = wrist;
    }

    public void setSlidePower(double power) {
        slidePower = power;
    }

    public void setArmPosition(/*position*/) {
        //maybe pidf stuff here?
    }

    public void setWrist(double position) {
        wrist.setPosition(position);
    }

    @Override
    public void periodic() {
        //TODO: make slide not reach too far, and also resets
        //do we reset every time we get to the bottom or not? non-limit switch button?
        slide1.setPower(slidePower);
        slide2.setPower(slidePower);
        //TODO: add pidf for arm servos

    }
}
