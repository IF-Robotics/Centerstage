package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClimbSubsystem extends SubsystemBase {
    private DcMotorEx motor;
    private Telemetry telemetry;
    private double power = 0;
    int MAX = 2400, MIN = 100;

    public ClimbSubsystem(DcMotorEx motor, Telemetry telemetry) {
        this.motor = motor;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.telemetry = telemetry;
    }

    //TODO: don't forget to to have ways to reset the slides

    public void setPower(double power) {
        this.power = power;
    }

    public void setPower(double power, Gamepad gamepad) {
        this.power = power * -1 * gamepad.right_stick_y;
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        //TODO: add stuff so that slides don't overreach up or down
        telemetry.addData("Climb", getPosition());
//        telemetry.update();

        if(getPosition() > MAX) {
            power = Math.min(0, power);
        } else if(getPosition() < MIN) {
            power = Math.max(0, power);
        }
        motor.setPower(power);
    }
}
