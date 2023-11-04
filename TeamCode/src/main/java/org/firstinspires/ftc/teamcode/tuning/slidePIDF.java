package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
@TeleOp(name="slidePIDF")

public class slidePIDF extends LinearOpMode {

    public static double p=0, i=0, d=0, f=0;
    private PIDFController controller = new PIDFController(p,i,d,f);
    public static int target = 0;
    private DcMotorEx slide1, slide2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide2.setDirection(DcMotorEx.Direction.REVERSE);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            controller.setPIDF(p, i, d, f);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            int slidePos = slide1.getCurrentPosition();
            double pidf = controller.calculate(slidePos, target);
            slide1.setPower(pidf);
            slide2.setPower(pidf);

            telemetry.addData("pos", slidePos);
            telemetry.addData("target", target);
            telemetry.addData("error", target - slidePos);
            telemetry.update();
        }
    }
}
