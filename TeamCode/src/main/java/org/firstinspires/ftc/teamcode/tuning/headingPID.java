package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="headingPID")
@Disabled
public class headingPID extends LinearOpMode {

    public double p=0, i=0, d=0;
    double target = 60;
    private PIDController thetaControl = new PIDController(p,i,d);
    private DcMotorEx BL, BR, FL, FR;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        imu.resetYaw();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            target = target + gamepad1.right_stick_x;
            if (target > 6.2832){
                target = 0;
            }

            if (target < 0){
                target = 6.2832;
            }

            if (gamepad1.touchpad) {
                imu.resetYaw();
            }

            thetaControl.setPID(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double t = thetaControl.calculate(botHeading/(Math.toRadians(target - botHeading)), -target/angleWrap(Math.toRadians(target - botHeading)));

            // this imaginary pid controller has a control method that uses the
            // PID controller we defined earlier in a method called calculate
            // the first argument of calculate is the reference state.
            // the second is the systems state.


            double x_rotated = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double y_rotated = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            // x, y, theta input mixing
            FL.setPower(x_rotated + y_rotated + t);
            BL.setPower(x_rotated - y_rotated + t);
            FR.setPower(x_rotated - y_rotated - t);
            BR.setPower(x_rotated + y_rotated - t);

            telemetry.addData("heading", botHeading * 57.2957795);
            telemetry.addData("target", target * 57.2957795);
            telemetry.addData("error", 57.2957795 * (target - botHeading));
            telemetry.update();
        }
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
}
