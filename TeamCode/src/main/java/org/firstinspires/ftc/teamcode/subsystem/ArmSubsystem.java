package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private DcMotorEx slide1, slide2;
    private CRServo arm1, arm2;
    private Servo wrist;
    private double position = 0, setpoint = 0; //TODO: find the actual value of this
    private double slidePower;
    private AnalogInput armInput;
    static double kp = .1, ki = 0, kd = 0, kf = 0;
    private PIDFController pidf = new PIDFController(kp, ki, kd, kf);

    public ArmSubsystem(DcMotorEx slide1, DcMotorEx slide2, CRServo arm1, CRServo arm2, AnalogInput input, Servo wrist) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.arm1 = arm1;
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.arm2 = arm2;
        this.armInput = input;
        this.wrist = wrist;
    }

    public void setSlidePower(double power) {
        slidePower = power;
    }

    public void setArmPosition(double setpoint) {
        this.setpoint = setpoint;
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
        position = armInput.getVoltage() / 3.3 * 360;
        arm1.setPower(pidf.calculate(position, setpoint));
        arm1.setPower(pidf.calculate(position, setpoint));
    }
}
