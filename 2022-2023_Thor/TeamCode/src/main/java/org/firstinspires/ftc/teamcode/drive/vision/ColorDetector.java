package org.firstinspires.ftc.teamcode.drive.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class ColorDetector extends LinearOpMode {
   WebcamName webcamName;

   public static int UPPER_H = 255;
   public static int UPPER_S = 255;
   public static int UPPER_V = 255;
   public static int LOWER_H = 0;
   public static int LOWER_S = 0;
   public static int LOWER_V = 0;

   @Override
   public void runOpMode() throws InterruptedException {
      webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

      // Adds live view port for the driver hub
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
      FtcDashboard.getInstance().startCameraStream(camera, 0);

      ColorScanner pipeline = new ColorScanner(telemetry);
      camera.setPipeline(pipeline);

      // Opening async. to avoid the thread from waiting for camera
      camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
      {
         @Override
         public void onOpened()
         {
            camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
         }
         @Override
         public void onError(int errorCode)
         {
            /*
             * This will be called if the camera could not be opened
             */
         }
      });

      telemetry.addLine("Initialized Camera");
      telemetry.update();
      waitForStart();
      while (opModeIsActive()) {
         pipeline.setUpperColor(UPPER_H, UPPER_S, UPPER_V);
         pipeline.setLowerColor(LOWER_H, LOWER_S, LOWER_V);
         telemetry.update();
      }
   }
}