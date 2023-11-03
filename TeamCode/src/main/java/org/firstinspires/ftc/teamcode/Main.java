package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
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
    public Servo inServo, wrist, drone, Uclaw, Lclaw;
    public Servo arm1, arm2;
    AnalogInput analogInput;
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

        //imu
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //drive subsystem
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveSubsystem = new DriveSubsystem(BL, BR, FL, FR, imu, telemetry);
        scheduler.registerSubsystem(driveSubsystem);

        //intake subsystem
        inMotor = hardwareMap.get(DcMotorEx.class, "inMotor");
        inMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inServo = hardwareMap.get(Servo.class, "inServo");
        intakeSubsystem = new IntakeSubsystem(inMotor, inServo, telemetry);
        scheduler.registerSubsystem(intakeSubsystem);

        //claw subsystem

        Uclaw = hardwareMap.get(Servo.class, "Uclaw");
        Lclaw = hardwareMap.get(Servo.class, "Lclaw");
        clawSubsystem = new ClawSubsystem(Uclaw, Lclaw, telemetry);
        scheduler.registerSubsystem(clawSubsystem);

        //climb subsysem
        climb = hardwareMap.get(DcMotorEx.class, "climb");
        climbSubsystem = new ClimbSubsystem(climb, telemetry);
        scheduler.registerSubsystem(climbSubsystem);

        //arm subsysem
        arm1 = hardwareMap.get(Servo.class, "Rarm");
        arm2 = hardwareMap.get(Servo.class, "Larm");
        ArmSubsystem.controller = new PIDController(ArmSubsystem.kp, ArmSubsystem.ki, ArmSubsystem.kd);
        analogInput = hardwareMap.get(AnalogInput.class, "armAnalog");
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armSubsystem = new ArmSubsystem(slide1, slide2, arm1, arm2, analogInput, wrist, telemetry);
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
