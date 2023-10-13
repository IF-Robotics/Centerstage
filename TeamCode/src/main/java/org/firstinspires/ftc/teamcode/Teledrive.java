package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Teledrive extends CommandBase {
    DriveSubsystem driveSubsystem;
    private double power = 0;
    private Gamepad gamepad1;

    public Teledrive(DriveSubsystem driveSubsystem, Gamepad gamepad1, double power) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        this.power = power;
        this.gamepad1 = gamepad1;
    }

    @Override
    public void initialize() {
        driveSubsystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void execute() {
        driveSubsystem.teleDrive(gamepad1, power);
    }

    @Override
    public void end(boolean interrupted) {

    }
}