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

    public static double p=0.003, i=0.1, d=0.0001, f=0.001;
    private PIDFController controller = new PIDFController(p,i,d,f);
    public static int target = 0;
    private DcMotorEx slide1, slide2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide1.setDirection(DcMotorEx.Direction.REVERSE);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            controller.setPIDF(p, i, d, f);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            int slidePos = slide1.getCurrentPosition();
            double pidf = controller.calculate(slidePos, target);
            if(target < 50) {
                slide1.setPower(0);
                slide2.setPower(0);
            } else{ p = 0.003;
                slide1.setPower(pidf);
                slide2.setPower(pidf);
            }

            telemetry.addData("pos", slidePos);
            telemetry.addData("power", pidf);
            telemetry.addData("target", target);
            telemetry.addData("error", Math.abs(target - slidePos));
            telemetry.update();
        }
    }
}
