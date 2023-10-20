package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AirplaneSubsystem extends SubsystemBase {
    private ServoEx servo;
    private Telemetry telemetry;
    public double reset = 0, shoot = 1; //TODO: make real values

    public AirplaneSubsystem(ServoEx servo, Telemetry telemetry) {
        this.servo = servo;
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
