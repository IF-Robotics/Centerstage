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
    /*private OpenCvCamera.AsyncCameraOpenListener listener = new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            autoCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        }
        @Override
        public void onError(int errorCode) {
        }
    };*/

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName AName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera autoCam = OpenCvCameraFactory.getInstance().createWebcam(AName, cameraMonitorViewId);
        autoCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                autoCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        this.aPipe = new PropPipeline(true);
        autoCam.setPipeline(aPipe);

        autoCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                autoCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            sleep(20);
            telemetry.addData("position", aPipe.getPosition());
            telemetry.addData("x-value", aPipe.getX());
            telemetry.addData("y-value", aPipe.getY());
            telemetry.addData("max area", aPipe.getArea());
            telemetry.update();
        }
    }
}
