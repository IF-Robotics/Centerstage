package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class TeledriveCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    private double power = .7;
    private double tempPower = 0;
    private Gamepad gamepad1;

    public TeledriveCommand(DriveSubsystem driveSubsystem, Gamepad gamepad1) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        this.gamepad1 = gamepad1;
    }

    @Override
    public void initialize() {
        driveSubsystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tempPower = power;
    }

    @Override
    public void execute() {
        if(gamepad1.left_bumper) {
            tempPower = power + .2;
        } else if(gamepad1.right_bumper) {
             tempPower = power - .2;
        }
        driveSubsystem.teleDrive(gamepad1, tempPower, true);
    }

    @Override
    public void end(boolean interrupted) {

    }
}