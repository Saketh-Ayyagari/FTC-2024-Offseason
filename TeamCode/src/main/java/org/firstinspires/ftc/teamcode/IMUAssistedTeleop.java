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

/**
 * Saketh Ayyagari
 * IMU-assisted controller using a finite State machine
 * Inspiration taken from 8/24 workshop
 **/
@TeleOp(name="IMUAssistedTeleop", group="Iterative Opmode")
@Disabled
public class IMUAssistedTeleop extends OpMode{
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot drivetrain;
    private IMU.Parameters myIMUParameters;
    private IMU imu;
    // PID Values
    private final double Kp = 0.03125;
    private final double Ki = 0;
    private final double Kd = 0;
    private Double prevError = 0.0;
    private double error_sum = 0;
    /**
    Two states in this state machine
     1. "lock": this uses a Proportional-Derivative controller to lock the robot's heading
     2. "unlock": this disables the controller and allows the robot to rotate in place
     **/
    private String state = "lock";

    private static final double MAX_POWER = 0.4;

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

        drivetrain = new Robot(hardwareMap);
        drivetrain.init();
        // initializing IMU
        imu.initialize(myIMUParameters);

    }
    public void start() {
        runtime.reset();
    }
    public void loop(){
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Create an object to receive the IMU angles
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //Flipped x and y because motors are flipped - 12/16
        double drive = gamepad1.left_stick_y; //controls drive by moving up or down.
        double turn = gamepad1.right_stick_x; // controls turning
        double strafe = gamepad1.left_stick_x; // controls strafing

        if (turn != 0){
            state = "unlock";
        }
        else{
            state = "lock";
        }

        telemetry.addData("Drive Power: ", drive);
        telemetry.addData("Turning Value: ", turn);
        telemetry.addData("Strafing Value: ", strafe);
        telemetry.addLine();

        leftPower = Range.clip(drive - turn, -MAX_POWER, MAX_POWER);
        rightPower = Range.clip(drive + turn, -MAX_POWER, MAX_POWER);


        // receiving IMU Angular Velocity Values
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        double heading = orientation.getYaw(AngleUnit.DEGREES);

        double SETPOINT = 0.0;
        double rotation_speed = PIDControl(SETPOINT, heading);

        drivetrain.rotate(rotation_speed);


        telemetry.addData("frontLeftPower", drivetrain.frontLeft.getPower());
        telemetry.addData("frontRightPower", drivetrain.frontRight.getPower());
        telemetry.addData("backLeftPower", drivetrain.backLeft.getPower());
        telemetry.addData("backRightPower", drivetrain.backRight.getPower());
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
        double D_error = Kd * (error - prevError)/runtime.seconds();
        prevError = error;
        // resets timer for recalculating derivative error
        resetRuntime();

        return Range.clip(P_error + D_error, -MAX_POWER, MAX_POWER);
    }
}

