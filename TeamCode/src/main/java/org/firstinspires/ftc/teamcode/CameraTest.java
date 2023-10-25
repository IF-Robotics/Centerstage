package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class CameraTest extends LinearOpMode {

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.enableLiveView(true);
        visionPortal = builder.build();

        while(opModeIsActive()) {
            sleep(20);
            visionPortal.resumeStreaming();
        }
        visionPortal.close();
    }
}
