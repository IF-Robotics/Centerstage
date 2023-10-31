package org.firstinspires.ftc.teamcode.subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.degrees;
@Config
public class ArmSubsystem extends SubsystemBase {
    private DcMotorEx slide1, slide2;
    private CRServo arm1, arm2;
    private Servo wrist;
    private int position = 0, setpoint = 0; //TODO: find the actual value of this
    private double slidePower;
    private AnalogInput armInput;
    public static double kp = .05, ki = 0, kd = 0;
    public static double kf = 0;
    public static PIDController controller;

    public static FtcDashboard dashboard;

    public static int target = 180;
    private Telemetry telemetry;

    public ArmSubsystem(DcMotorEx slide1, DcMotorEx slide2, CRServo arm1, CRServo arm2, AnalogInput input, Servo wrist, Telemetry telemetry) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.arm1 = arm1;
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.arm2 = arm2;
        this.armInput = input;
        this.wrist = wrist;
        this.telemetry = telemetry;
    }

    public void setSlidePower(double power) {
        slidePower = power;
    }

    public void setArmPosition(double setpoint) {
        this.setpoint = (int) setpoint;
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
        position = (int) (armInput.getVoltage() / 3.3 * 360);
        controller.setPID(kp, ki, kd);
        double pid = controller.calculate(position, target);
        double ff = Math.cos(Math.toRadians(target / 1)) * kf;

        double power = pid + ff;

        arm1.setPower(power);

        telemetry.addData("pos", position);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.update();
    }
}
