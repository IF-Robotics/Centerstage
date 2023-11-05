package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AirplaneSubsystem extends SubsystemBase {
    private Servo servo;
    private Telemetry telemetry;
    public double reset = .55, shoot = .86; //TODO: make real values

    public AirplaneSubsystem(Servo servo, Telemetry telemetry) {
        this.servo = servo;
//        servo.setPosition(reset);
        this.telemetry = telemetry;
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void periodic() {

    }
}
