package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DriveSubsystem extends SubsystemBase {

    private DcMotorEx BL, BR, FL, FR;
    private Telemetry telemetry;

    public enum Direction {
        left,
        right,
        forward,
        backward,
        auto
    }
    private Direction dir = Direction.forward;

    private double power = 0;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
    private IMU imu;

    public DriveSubsystem(DcMotorEx BL, DcMotorEx BR, DcMotorEx FL, DcMotorEx FR, IMU imu, Telemetry telemetry) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;
        this.imu = imu;
        this.telemetry = telemetry;

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        this.runMode = runMode;
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void drive(double power, Direction dir) {
        this.power = power;
        this.dir = dir;

        if(dir == Direction.backward || dir == Direction.left) {
            this.power *= -1;
        }
    }

    public void teleDrive(Gamepad gamepad1, double power) {
        double LY = (double) gamepad1.left_stick_y;
        double LX = (double) gamepad1.left_stick_x;
        double RY = (double) gamepad1.right_stick_y;
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        setAllPower(
                power * (LY + LX),
                power * (RY - LX),
                power * (LY - LX),
                power * (RY + LX)
        );
    }

    //field centric
    public boolean teleDrive(Gamepad gamepad1, double power, boolean isFieldCentric) {
        if(!isFieldCentric) {
            teleDrive(gamepad1, power);
            return false;
        }

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = Math.sqrt(Math.pow(gamepad1.right_stick_x,2) * Math.pow(gamepad1.right_stick_y,2));

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        setAllPower(backLeftPower, backRightPower, frontLeftPower, frontRightPower);
        return true;
    }

    private void setAllPower(double BL, double BR, double FL, double FR) {
        this.BL.setPower(BL);
        this.BR.setPower(BR);
        this.FL.setPower(FL);
        this.FR.setPower(FR);
        dir = Direction.auto;
    }

    @Override
    public void periodic() {

        if(BL.getMode() != runMode) {
            BL.setMode(runMode);
            BR.setMode(runMode);
            FL.setMode(runMode);
            FR.setMode(runMode);
        }

        if(runMode == DcMotor.RunMode.RUN_USING_ENCODER) {
            if(dir == Direction.left || dir == Direction.right) {
                BL.setPower(power);
                BR.setPower(-power);
                FL.setPower(-power);
                FR.setPower(power);
            } else if (dir == Direction.backward || dir == Direction.forward){
                BL.setPower(power);
                BR.setPower(power);
                FL.setPower(power);
                FR.setPower(power);
            } else {
                //manual control is going
                telemetry.addLine("auto control");
            }
        }

        telemetry.addData("drive amps", BL.getCurrent(CurrentUnit.AMPS) + BR.getCurrent(CurrentUnit.AMPS) + FL.getCurrent(CurrentUnit.AMPS) + FR.getCurrent(CurrentUnit.AMPS));
    }
}

