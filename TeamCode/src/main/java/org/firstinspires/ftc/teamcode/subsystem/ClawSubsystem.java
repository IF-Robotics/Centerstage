package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase {
    private Servo upper, lower;
    public double openL = .65, closeL = .45, openU = .35, closeU = .5;
    private boolean isGoalOpen = false, isOpenRn = false;
    private Telemetry telemetry;
    //lower is .45 for close and .65 for open
    //upper is .5 for open and .35 for close

    public ClawSubsystem(Servo upper, Servo lower, Telemetry telemetry) {
        this.upper = upper;
        this.lower = lower;
        setLower(closeL);
        setUpper(closeU);
        this.telemetry = telemetry;
    }

    public void open() {
            setLower(openL);
            setUpper(openU);
    }

    public void close() {
        setLower(closeL);
        setUpper(closeU);
    }

    public void setLower(double position) {
        lower.setPosition(position);
    }
    public void setUpper(double position) {
        upper.setPosition(position);
    }

    @Override
    public void periodic() {
        /*if(isGoalOpen != isOpenRn) {
            if(isGoalOpen) {
                setLower(openL);
                setUpper(openU);
                isOpenRn = true;
            } else {
                setLower(closeL);
                setUpper(closeU);
                isOpenRn = false;
            }
        }*/
//        telemetry.addData("lower", lower.getPosition())
    }
}
