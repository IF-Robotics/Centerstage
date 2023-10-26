package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotorEx motor;
    private Servo servo;
    private Telemetry telemetry;
    public double up = .5, down = .27; //TODO: make these actually the real values

    public IntakeSubsystem(DcMotorEx motor, Servo servo, Telemetry telemetry) {
        this.motor = motor;
        servo.setPosition(up);
        this.servo = servo;
        this.telemetry = telemetry;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    @Override
    public void periodic() {

    }
}
