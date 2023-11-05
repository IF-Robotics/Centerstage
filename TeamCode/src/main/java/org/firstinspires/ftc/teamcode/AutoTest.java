package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.stat.descriptive.rank.Percentile;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

@Autonomous
public class AutoTest extends CommandOpMode {

    double integralSum = 0;
    double Kp=2,Ki=.2,Kd=0;
    double referenceAngle;
    double heading;
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    IMU imu;

    DcMotorEx BL, BR, FL, FR;
    Servo arm1, arm2, wrist, uClaw, intake;
Main robot;
Command armDown;
PropPipeline.Position position = PropPipeline.Position.unknown;
    @Override
    public void initialize() {
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        uClaw = hardwareMap.get(Servo.class, "Uclaw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm1 = hardwareMap.get(Servo.class, "Rarm");
        arm2 = hardwareMap.get(Servo.class, "Larm");
        intake = hardwareMap.get(Servo.class, "inServo");

        robot = new Main("auto", hardwareMap, telemetry);
        configureCommands();

        arm1.setPosition(.891);
        arm2.setPosition(.891);

        uClaw.setPosition(.35);
        intake.setPosition(.5);

        while(!isStarted()) {
            telemetry.addData("position", robot.aPipe.getPosition());
            telemetry.addData("x-value", robot.aPipe.getX());
            telemetry.addData("y-value", robot.aPipe.getY());
            telemetry.addData("max area", robot.aPipe.getMaxContour());
            telemetry.update();
        }

        waitForStart();
        imu.resetYaw();
        position = robot.aPipe.getPosition();
        wrist.setPosition(.717);
        arm1.setPosition(.0825);
        arm2.setPosition(.0825);
        arm1.setPosition(.0825);
        arm2.setPosition(.0825);
        drive(32000);
        if(position == PropPipeline.Position.right) {
            Kp = 3;

            turn(-60);

        } else if(position == PropPipeline.Position.center) {

        } else {
            Kp = 3;

            turn(60);

        }
        uClaw.setPosition(.5);
        sleep(1000);
        wrist.setPosition(.6);
        arm1.setPosition(.6);
        arm2.setPosition(.6);
    }



    private void configureCommands() {
        //create your commands here
        //for example, to drive, for powers of .7 with multiplier of 1
//        robot.driveSubsystem.setAllPower(.7, .7, .7, .7, 1);

        //to set runmode use
//        robot.driveSubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void turn(int target){
        ElapsedTime timer1 = new ElapsedTime();
        timer1.reset();
        while(timer1.milliseconds() < 3000) {
            referenceAngle = Math.toRadians(target);
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double power = PIDcontrol(referenceAngle, heading);

            FL.setPower(-power);
            BL.setPower(-power);
            BR.setPower(power);
            FR.setPower(power);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            telemetry.addData("pose", heading);
            telemetry.addData("referance", referenceAngle);
            telemetry.addData("error(degrees)", Math.toDegrees(referenceAngle - heading));
            telemetry.update();
        }
    }

    public double PIDcontrol(double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    public void drive(int distance){


        referenceAngle = Math.toRadians(0);
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double power = PIDcontrol(referenceAngle, heading);

        while(-BR.getCurrentPosition() < distance){
            FL.setPower(.3 + power);
            BL.setPower(.3 + power);
            BR.setPower(.3 - power);
            FR.setPower(.3 - power);
        }
        telemetry.addData("distanceForward", BR.getCurrentPosition());
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

}
