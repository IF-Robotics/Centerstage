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
    public double armDown = .88, armUp = .18, armGround, armNeutral = .75, armFlip = .89;
    private Servo wrist;
    public double wristUp = .48, wristDown = .8, wristGround, wristNeutral, wristFlip = .17;
    private int position = 0, setpoint = 0; //TODO: find the actual value of this
    private PController slideController = new PController(.01);
    private double slidePower = .6;
    private int slidePosition;
    private AnalogInput armInput;
    public static double kp = .025, ki = 0.05, kd = 0.001;
    public static double kf = 0;
    public static PIDController controller;

    public static FtcDashboard dashboard;
    public static int target = 180;
    private Telemetry telemetry;

    public ArmSubsystem(DcMotorEx slide1, DcMotorEx slide2, Servo arm1, Servo arm2, AnalogInput input, Servo wrist, Telemetry telemetry) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.arm1 = arm1;
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.arm2 = arm2;
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
        slide1.setPower(slidePower);
        slide2.setPower(slidePower);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        //TODO: add pidf for arm servos
//        position = (int) (armInput.getVoltage() * (360 / 3.3));
//        controller.setPID(kp, ki, kd);
//        double pid = controller.calculate(position, target);
//        double ff = Math.cos(target-128) * kf;
//
//        double power = pid + ff;
//
//        arm1.setPower(power/5.0);
//        arm2.setPower(power/5.0);


        telemetry.addData("pos", position);
        telemetry.addData("target", target);
        /*telemetry.addData("power", power);
        telemetry.addData("f", ff);*/
        telemetry.addData("arm power",arm1.getPosition());
        telemetry.addData("wrist", wrist.getPosition());
        telemetry.update();
    }
}
