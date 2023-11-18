package org.firstinspires.ftc.teamcode.subsystem;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.degrees;
@Config
public class ArmSubsystem extends SubsystemBase {
    private DcMotorEx slide1, slide2;
    private Servo arm1, arm2;
    public double armDown = .869, armUp = .159, armGround, armNeutral = .75, armFlip = .892;
    //armup was .285
    private Servo wrist;
    public double wristDown = .811, wristUp = .475, wristGround, wristNeutral, wristFlip = .169;
    //wristup was .48
    private int position = 0, setpoint = 0; //TODO: find the actual value of this
    public static FtcDashboard dashboard;
    private double p=0.003, i=0.2, d=0.0001, f=0.001;
    private PIDFController slideController = new PIDFController(p,i,d,f);
    private double slidePower = 1;
    public static int slidePosition = 0;
    private AnalogInput armInput;
    private Telemetry telemetry;

    public ArmSubsystem(DcMotorEx slide1, DcMotorEx slide2, Servo arm1, Servo arm2, AnalogInput input, Servo wrist, Telemetry telemetry) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.arm1 = arm1;
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.arm2 = arm2;
        arm1.setPosition(armFlip);
        arm2.setPosition(armFlip);
        wrist.setPosition(wristFlip);
        this.armInput = input;
        this.wrist = wrist;
        this.telemetry = telemetry;
    }

    public void setArm(double target) {
        arm1.setPosition(target);
        arm2.setPosition(target);
    }

    public void setSlidePosition(int slidePosition) {
        this.slidePosition = slidePosition;
    }

    public void addSlidePosition(int add) {
        slidePosition +=  add;
    }
    public void setSlidePower(double power) {
        slidePower = power;
    }

    //depreciatied
//    public void setArmPosition(double setpoint) {
//        this.setpoint = (int) setpoint;
//    }

    public void setWrist(double position) {
//        double adjPos = Math.min(.7, position);
        wrist.setPosition(position);
    }

    public double getWrist() {
        return wrist.getPosition();
    }

    @Override
    public void periodic() {
        //TODO: make slide not reach too far, and also resets
        //do we reset every time we get to the bottom or not? non-limit switch button?
        slidePower = slideController.calculate(slide1.getCurrentPosition(), slidePosition);
        if(slide1.getCurrentPosition() < 10) {
            slidePower = Math.max(0, slidePower);
        }
        if(slidePosition < 50) {
            slide1.setPower(0);
            slide2.setPower(0);
        } else {
            slide1.setPower(slidePower);
            slide2.setPower(slidePower);
        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        telemetry.addData("ArmPos", position);
        telemetry.addData("slidePower",slidePower);
        /*telemetry.addData("power", power);
        telemetry.addData("f", ff);*/
        telemetry.addData("wrist", wrist.getPosition());
        telemetry.update();
    }
}
