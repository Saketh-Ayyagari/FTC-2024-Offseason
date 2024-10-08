package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;


@Autonomous(name="Contour Detection", group="Iterative OpMode")
//@Disabled
public class ContourDetection extends OpMode{

   // Declare OpMode members for each of the 4 motors.
   private ElapsedTime runtime = new ElapsedTime();
   private DcMotor frontLeft;
   private DcMotor backLeft;
   private DcMotor frontRight;
   private DcMotor backRight;

   // camera variables
   private static final int CAMERA_WIDTH = 640;
   private static final int CAMERA_HEIGHT = 480;

   private int cameraMonitorViewId;
   private OpenCvCamera camera;
   private WebcamName webcamName;
   private ContourDetectionPipeline pipeline;
   private static double contour_area = 0;
   private static Point contour_center = new Point();

   FtcDashboard dashboard = FtcDashboard.getInstance();
   Telemetry telemetry = dashboard.getTelemetry(); // ONLY WORKS ON STATIC VARIABLES

   @Override
   public void init() {

      backRight = hardwareMap.dcMotor.get("backRight"); //port 3
      frontRight = hardwareMap.dcMotor.get("frontRight"); //port 2
      backLeft = hardwareMap.dcMotor.get("backLeft"); //port 1
      frontLeft = hardwareMap.dcMotor.get("frontLeft"); //port 0

      backRight.setDirection(DcMotor.Direction.REVERSE);
      frontRight.setDirection(DcMotor.Direction.REVERSE);

      // setting the mode of each motor to run without encoders
      frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      // initializing camera
      cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
              hardwareMap.appContext.getPackageName());
      webcamName = hardwareMap.get(WebcamName.class, "webcam13115");
      camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

      // initializing and setting pipeline for use in the loop() method
      ContourDetectionPipeline pipeline = new ContourDetectionPipeline();
      camera.setPipeline(pipeline);

      camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
         @Override
         public void onOpened(){ // what will happen when "Camera Stream" is clicked
            // Usually this is where you'll want to start streaming from the camera (see section 4)
            camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            // Streams camera to FTC Dashboard
            FtcDashboard.getInstance().startCameraStream(camera, 0);
            contour_center = pipeline.contour_center;
            contour_area = pipeline.contour_area;
            telemetry.addLine("Camera successfully initialized");
            telemetry.addData("Contour Center: ", contour_center.toString());
            telemetry.addData("Contour Area: ", contour_area);
            telemetry.update();
         }
         @Override
         public void onError(int errorCode){
            telemetry.addLine("Camera failed.");
            telemetry.update();
         }
      });


      // Wait for the game to start (driver presses PLAY)
      telemetry.addData("Status", "Initialized");
      telemetry.update();
   }
   @Override
   public void start(){
      //start
      backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      runtime.reset();
   }
   @Override
   public void loop(){
      telemetry.addData("Status", "Camera running for: " + runtime.toString());
      telemetry.update();
   }

}

