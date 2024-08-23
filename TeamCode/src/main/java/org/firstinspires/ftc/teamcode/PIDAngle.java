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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="PIDAngleControl", group="Linear Opmode")
//@Disabled
public class PIDAngle extends OpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private IMU.Parameters myIMUParameters;
    private IMU imu;
    // PID Values
    private double Kp = 0.03125;
    private double Ki = 0;
    private double Kd = 0;
    private Double prevError = null;
    private double error_sum = 0;

    @Override
    public void init(){
        // defining IMU parameters
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        );
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        //start
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // initializing IMU
        imu.initialize(myIMUParameters);
    }

    @Override
    public void start(){
        runtime.reset();
    }
    @Override
    public void loop() {
        // rc_control();
        // Create an object to receive the IMU angles
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // receiving IMU Angular Velocity Values
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        double heading = orientation.getYaw(AngleUnit.DEGREES);
        final double SETPOINT = 90.0;
        rc_control(6);
//        double rotation_speed = PIDControl(SETPOINT, heading);
//
//        rotate(rotation_speed);


        telemetry.addData("frontLeftPower", frontLeft.getPower());
        telemetry.addData("frontRightPower", frontRight.getPower());
        telemetry.addData("backLeftPower", backLeft.getPower());
        telemetry.addData("backRightPower", backRight.getPower());
        telemetry.addLine();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        telemetry.addLine();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();

    }
    public double PIDControl(double setpoint, double current){
        double error = setpoint - current;
        double P_error = Kp*error;
        // calculates derivative error
        double D_error = 0;
        if (prevError != null){
            D_error = Kd * (error - prevError)/runtime.seconds();
            resetRuntime();
            prevError = error;
        }
        error_sum += error;
        double I_error = Ki*error_sum;
        return Range.clip(P_error + I_error + D_error, -0.6, 0.6);
    }
    public void rc_control(double maxDrivePower){
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //Flipped x and y because motors are flipped - 12/16
        double drive = gamepad1.left_stick_y; //controls drive by moving up or down.
        double turn = gamepad1.left_stick_x;

        telemetry.addData("Drive Power: ", drive);
        telemetry.addData("Turning Value: ", turn);
        telemetry.addLine();

        leftPower = Range.clip(drive - turn, -maxDrivePower, maxDrivePower);
        rightPower = Range.clip(drive + turn, -maxDrivePower, maxDrivePower);

        // Send calculated power to wheels
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);

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

