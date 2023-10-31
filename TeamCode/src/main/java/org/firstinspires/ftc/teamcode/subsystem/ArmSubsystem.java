package org.firstinspires.ftc.teamcode.subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.arcrobotics.ftclib.controller.PIDController;

import com.arcrobotics.ftclib.controller.PController;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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


    private double position = 0, setpoint = 0; //TODO: find the actual value of this
    private PController slideController = new PController(.01);
    private double slidePower = .6;
    private int slidePosition;
    private AnalogInput armInput;
    static double kp = .1, ki = 0, kd = 0, kf = 0;
    private PIDFController pidf = new PIDFController(kp, ki, kd, kf);
    private Telemetry telemetry;


    public ArmSubsystem(DcMotorEx slide1, DcMotorEx slide2, CRServo arm1, CRServo arm2, AnalogInput input, Servo wrist, Telemetry telemetry) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void setSlidePosition(int slidePosition) {
        this.slidePosition = slidePosition;
    }

    public void addSlidePosition(int add) {
        slidePosition +=  add;
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
        slidePower = slideController.calculate(slide1.getCurrentPosition(), slidePosition);
        slide1.setPower(slidePower);
        slide2.setPower(slidePower);
        telemetry.addData("slide1", slide1.getPower());
        telemetry.addData("position", slide1.getCurrentPosition());
        telemetry.addData("target", slidePosition);
        telemetry.update();


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

        /*//TODO: add pidf for arm servos
        position = armInput.getVoltage() / 3.3 * 360;
        arm1.setPower(pidf.calculate(position, setpoint));
        arm1.setPower(pidf.calculate(position, setpoint));*/

    }
}
