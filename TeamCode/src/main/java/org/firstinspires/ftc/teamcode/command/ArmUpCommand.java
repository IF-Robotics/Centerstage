package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class ArmUpCommand extends CommandBase {
    ArmSubsystem arm;
    int slide, time;
    double armPos;
    double wrist;
    ElapsedTime timer;
    public ArmUpCommand(ArmSubsystem arm, int slide, double armPos, double wrist, int time) {
        this.arm = arm;
        this.slide = slide;
        this.armPos = armPos;
        this.wrist = wrist;
        this.time = time;
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.setArm(armPos);
        arm.setSlidePosition(slide);
    }

    @Override
    public void execute() {
        if(timer.milliseconds() > time) {
            arm.setWrist(wrist);
        }
    }
}

