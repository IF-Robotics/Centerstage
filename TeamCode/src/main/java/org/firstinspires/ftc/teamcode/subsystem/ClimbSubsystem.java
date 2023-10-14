package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClimbSubsystem extends SubsystemBase {
    private DcMotorEx motor;
    private Telemetry telemetry;

    public ClimbSubsystem(DcMotorEx motor, Telemetry telemetry) {
        this.motor = motor;
        this.telemetry = telemetry;
    }

    //TODO: don't forget to to have ways to reset the slides

    public void setPower(double power) {
        motor.setPower(power);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        //TODO: add stuff so that slides don't overreach up or down
    }
}
