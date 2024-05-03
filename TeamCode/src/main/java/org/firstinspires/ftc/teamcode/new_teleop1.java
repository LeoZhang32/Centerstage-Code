package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class new_teleop1 extends LinearOpMode{
    DcMotor intake;
    DcMotor rightViper;
    DcMotor leftViper;
    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontRightMotor;
    Servo claw;
    Servo claw_wrist;
    Servo claw_elbow;
    Servo drone;
    @Override
    public void runOpMode() throws InterruptedException{
        intake = hardwareMap.dcMotor.get("intake");
        leftViper = hardwareMap.dcMotor.get("leftViper");
        rightViper = hardwareMap.dcMotor.get("rightViper");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        claw = hardwareMap.servo.get("claw");
        claw_wrist = hardwareMap.servo.get("claw_wrist");
        claw_elbow = hardwareMap.servo.get("claw_elbow");
        drone = hardwareMap.servo.get("drone");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //TODO: change these parameters after control hub installation.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.a){
                intake.setPower(0.6);
            }
            else {
                intake.setPower(0);
            }
            if (gamepad1.dpad_up){
                leftViper.setPower(0.6);
                rightViper.setPower(-0.6);
            }
            else if (gamepad1.dpad_down) {
                leftViper.setPower(-0.6);
                rightViper.setPower(0.6);
            }
            else{
                leftViper.setPower(0);
                rightViper.setPower(0);
            }
            
        }
    }
}
