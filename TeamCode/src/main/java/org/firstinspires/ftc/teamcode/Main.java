package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Main {

    public DcMotorEx BL, BR, FL, FR;
    private HardwareMap hardwareMap;

    public DriveSubsystem driveSubsystem;

    public Main(/*Opmode mode*/ String mode, HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        if(/*mode == Opmode.Teleop*/ mode == "tele") {
            initTele(telemetry);
        } else {
            initAuto();
        }
    }

    private void initTele(Telemetry telemetry) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.reset();

        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        driveSubsystem = new DriveSubsystem(BL, BR, FL, FR, telemetry);
        scheduler.registerSubsystem(driveSubsystem);
        scheduler.run();
    }

    private void initAuto() {
        CommandScheduler schduler = CommandScheduler.getInstance();
    }
}
