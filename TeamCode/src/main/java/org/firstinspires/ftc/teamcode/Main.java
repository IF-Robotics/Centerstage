package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class Main {

    public DcMotorEx BL, BR, FL, FR, inMotor, slide1, slide2, climb;
    public Servo inServo, wrist, drone, Rclaw, Lclaw;
    public CRServo arm1, arm2;
    private HardwareMap hardwareMap;

    public DriveSubsystem driveSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public ClawSubsystem clawSubsystem;
    public ClimbSubsystem climbSubsystem;
    public ArmSubsystem armSubsystem;
    public AirplaneSubsystem airplaneSubsystem;

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

        //drive subsystem
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveSubsystem = new DriveSubsystem(BL, BR, FL, FR, telemetry);
        scheduler.registerSubsystem(driveSubsystem);

        //intake subsystem
        inMotor = hardwareMap.get(DcMotorEx.class, "inMotor");
        inServo = hardwareMap.get(Servo.class, "inServo");
        intakeSubsystem = new IntakeSubsystem(inMotor, inServo, telemetry);
        scheduler.registerSubsystem(intakeSubsystem);

        //claw subsystem
        Rclaw = hardwareMap.get(Servo.class, "Rclaw");
        Lclaw = hardwareMap.get(Servo.class, "Lclaw");
        clawSubsystem = new ClawSubsystem(Rclaw, Lclaw);
        scheduler.registerSubsystem(clawSubsystem);

        //climb subsysem
        climb = hardwareMap.get(DcMotorEx.class, "climb");
        climbSubsystem = new ClimbSubsystem(climb, telemetry);
        scheduler.registerSubsystem(climbSubsystem);

        //arm subsysem
        arm1 = hardwareMap.get(CRServo.class, "arm1");
        arm2 = hardwareMap.get(CRServo.class, "arm2");
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armSubsystem = new ArmSubsystem(slide1, slide2, arm1, arm2, wrist);
        scheduler.registerSubsystem(armSubsystem);

        //airplane subsystem
        drone = hardwareMap.get(Servo.class, "plane");
        airplaneSubsystem = new AirplaneSubsystem(drone, telemetry);
        scheduler.registerSubsystem(airplaneSubsystem);

        scheduler.run(); //does this need to be here?
    }

    private void initAuto() {
        CommandScheduler schduler = CommandScheduler.getInstance();
    }
}
