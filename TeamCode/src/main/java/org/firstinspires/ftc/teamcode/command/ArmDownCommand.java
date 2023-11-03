package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;

public class ArmDownCommand extends CommandBase {
    ArmSubsystem arm;
    ClawSubsystem claw;
    int slide, time;
    double armPos;
    double wrist;
    ElapsedTime timer;
    public ArmDownCommand(ArmSubsystem arm, ClawSubsystem claw, int slide, double armPos, double wrist, int time) {
        this.arm = arm;
        this.claw = claw;
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
        claw.setUpper(claw.closeU);
        claw.setLower(claw.closeL);
    }

    @Override
    public void execute() {
        if(timer.milliseconds() > 400) {
            arm.setArm(armPos);
        }
        if(timer.milliseconds() > time) {
            arm.setWrist(wrist);
            arm.setSlidePosition(slide);
        }
    }
}
