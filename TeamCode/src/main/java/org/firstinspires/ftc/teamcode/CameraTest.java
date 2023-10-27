package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class CameraTest extends LinearOpMode {
    private PropPipeline aPipe;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName AName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera autoCam = OpenCvCameraFactory.getInstance().createWebcam(AName, cameraMonitorViewId);
        autoCam.openCameraDevice();
        this.aPipe = new PropPipeline();
        autoCam.setPipeline(aPipe);
        startCamera(autoCam);
        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            sleep(20);
            telemetry.addData("x-value", aPipe.getX());
            telemetry.addData("y-value", aPipe.getY());
            telemetry.update();
        }
    }

    public void startCamera(OpenCvCamera wcam) {
        if (!isStopRequested()) {
            try {
                wcam.openCameraDevice();
                wcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
            catch (Exception e) {
                telemetry.addData("error",e);
                telemetry.update();
                startCamera(wcam);
            }
        }
    }
}
