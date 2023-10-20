package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public DriveSubsystem(DcMotorEx BL, DcMotorEx BR, DcMotorEx FL, DcMotorEx FR, Telemetry telemetry) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;
        this.telemetry = telemetry;

    }

    public void setRunMode(DcMotor.RunMode runMode) {
        this.runMode = runMode;
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
        //TODO: add field centric code here
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
        telemetry.addLine("periodic running");

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

