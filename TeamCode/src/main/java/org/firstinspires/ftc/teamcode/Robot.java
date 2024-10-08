package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Saketh Ayyagari
 * "Robot" class for motor control
 */
public class Robot{
    private HardwareMap hardwareMp;
    // Declare OpMode members for each of the 4 motors.
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    // maximum power robot can drive
    public final double MAX_POWER = 0.6;

    private Telemetry telemetry;

    // initializes robot motors, encoders, etc. MUST be run before any movement occurs
    // the init method must be the one to take in a
    public Robot(HardwareMap hwMp){
        this.hardwareMp = hwMp;
    }
    public void init(){
        // initializes all motors
        backRight = hardwareMp.get(DcMotor.class, "backRight"); //port 3
        frontRight = hardwareMp.get(DcMotor.class, "frontRight"); //port 2
        backLeft = hardwareMp.get(DcMotor.class, "backLeft"); //port 1
        frontLeft = hardwareMp.get(DcMotor.class, "frontLeft"); //port 0

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
    /*
     * Negative Power = strafe left
     * Positive power = strafe right
     * */
    public void strafe(double power){
        frontLeft.setPower(-power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(-power);
    }
    public void brake(){
        backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }
    // given parameters for speed, angle, and strafe power, send power to the motors
    // inspired by MIT RACECAR system where it sends (speed, angle) to the car
    public void powerMotors(double drive, double turn, double strafe){
        double leftPower = Range.clip(drive - turn, -MAX_POWER, MAX_POWER);
        double rightPower = Range.clip(drive + turn, -MAX_POWER, MAX_POWER);

        // Send calculated power to wheels
        frontLeft.setPower(Range.clip(leftPower-strafe, -MAX_POWER, MAX_POWER));
        frontRight.setPower(Range.clip(rightPower+strafe, -MAX_POWER, MAX_POWER));
        backLeft.setPower(Range.clip(leftPower+strafe, -MAX_POWER, MAX_POWER));
        backRight.setPower(Range.clip(rightPower-strafe, -MAX_POWER, MAX_POWER));
    }

}

