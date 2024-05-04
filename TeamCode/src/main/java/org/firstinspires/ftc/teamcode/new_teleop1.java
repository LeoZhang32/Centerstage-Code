package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class new_teleop1 extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
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
    Servo intake_drop;
    //booleans
    String[] wrist_positions = {"0", "1", "2", "3"};
    Integer x = 0;
    String wrist_position = wrist_positions[x];
    boolean wrist_is_0 = true;
    Boolean wrist_is_1 = false;
    Boolean wrist_is_2 = false;
    Boolean wrist_is_3 = false;
    Boolean dpad_right_pressed = false;
    Boolean dpad_left_pressed = false;
    Boolean elbow_position_score = false;
    Boolean elbow_button_pressed = false;
    Boolean claw_open = true;
    Boolean claw_button_pressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
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
        intake_drop = hardwareMap.servo.get("intake_drop");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.left_bumper) {
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

            frontLeftMotor.setPower(0.75 * frontLeftPower);
            backLeftMotor.setPower(0.75 * backLeftPower);
            frontRightMotor.setPower(0.75 * frontRightPower);
            backRightMotor.setPower(0.75 * backRightPower);

            if (gamepad1.a) {
                intake.setPower(0.6);
                intake_drop.setPosition(1);
            } else {
                intake.setPower(0);
                intake_drop.setPosition(0.5);
            }
            if (gamepad1.dpad_up) {
                leftViper.setPower(1);
                rightViper.setPower(-1);
            } else if (gamepad1.dpad_down) {
                leftViper.setPower(-1);
                rightViper.setPower(1);
            } else {
                leftViper.setPower(0);
                rightViper.setPower(0);
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                if (gamepad1.dpad_right){
                    x += 1;
                    wrist_position = wrist_positions[(int) x];
                }
            }
            claw_elbow.setPosition(0);
            updateBooleans();
        }
    }
    public void updateBooleans() {
        if (wrist_position.equals(String.valueOf(0))) {
            claw_wrist.setPosition(0);
        }
        else if (wrist_position.equals(String.valueOf(1))) {
            claw_wrist.setPosition(0.2);
        }
        else if (wrist_position.equals(String.valueOf(2))) {
            claw_wrist.setPosition(0.4);
        }
        else if (wrist_position.equals(String.valueOf(3))){
            claw_wrist.setPosition(0.6);
        }
    }
}
