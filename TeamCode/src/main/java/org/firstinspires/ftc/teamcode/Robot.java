package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class Robot{
    private HardwareMap hardwareMap;

    // Declare OpMode members for each of the 4 motors.
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    // IMU and IMU Parameters
    private IMU.Parameters myIMUParameters; private IMU imu;
    // variables for camera use
    private int cameraMonitorViewId;
    private OpenCvCamera camera;
    private WebcamName webcamName;
    // camera resolution variables
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private Telemetry telemetry;
    public Robot(HardwareMap hardware){
        this.hardwareMap = hardware;
    }
    // initializes robot motors, encoders, etc. MUST be run before any movement occurs
    public void init(){
        // initializes all motors
        backRight = hardwareMap.dcMotor.get("backRight"); //port 3
        frontRight = hardwareMap.dcMotor.get("frontRight"); //port 2
        backLeft = hardwareMap.dcMotor.get("backLeft"); //port 1
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); //port 0
        imu = hardwareMap.get(IMU.class, "imu");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        // setting the mode of each motor to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //start at 0 power
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    // initializes camera to use EasyOpenCV
    public void initOpenCV(){
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
                telemetry.addLine("Camera successfully initialized");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode){
                telemetry.addLine("Camera failed.");
                telemetry.update();
            }
        });
    }
    public void forward(double power){
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    public void backward(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    /*
    * Positive power = rotate counterclockwise (causes heading to become larger)
    * Negative power = rotate clockwise (causes heading to become smaller)
    * Just like unit circle
    * */
    public void rotate(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    public void brake(){
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
}

