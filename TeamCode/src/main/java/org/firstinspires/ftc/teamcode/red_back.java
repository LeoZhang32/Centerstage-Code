package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class red_back extends LinearOpMode {
    Servo claw;
    Servo clawPivot;
    Servo droneLauncher;
    DcMotor intake;
    DcMotor ViperSlideRight;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("intake");
        ViperSlideRight = hardwareMap.dcMotor.get("ViperSlideRight");
        claw = hardwareMap.servo.get("claw");
        clawPivot = hardwareMap.servo.get("clawPivot");
        droneLauncher = hardwareMap.servo.get("droneLauncher");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //modify this yourself base on what direction each wheel is spinning

        waitForStart();

        if (opModeIsActive()){ //auto code
// since we start with 2 pixels in the claw, with the one we put on the line on the bottom
//we drive on the line, release claw, move viper slide up a tiny bit, close claw so it grabs the one on top, then move it up more
            clawClose();
            clawPivotNormal();
            sleep(500);
            moveForward(0.25,750);
            turnLeft(0.25,2000); //90 degrees + a bit more
            stopMotors();
            moveBack(0.25, 9000) ;//2500*2.5
            stopMotors();
            moveLeft(0.25,2000);
            stopMotors();
            clawOpen();

//put top pixels on board
//            clawOpen();
//            sleep(500);
//            ViperUp(60);
//            stopMotors();
//            moveForward(0.25,100);
//            stopMotors();
//            putOnBoard();
//            //second pixel
//            moveBack(0.25,100);
//            ViperDown(2000);
//            stopMotors();
//            putOnBoard();
            //park
        }
    }
    public void putOnBoard(){
        sleep(500);
        clawClose();
        sleep(500);
        ViperUp(2500);
        stopMotors();
        sleep(500);
        clawPivotScoring();
        sleep(500);
        clawOpen();
        sleep(500);
        clawPivotNormal();
        sleep(500);
        ViperDown(2500);
        stopMotors();
        sleep(500);
    }
    public void moveForward (double power, long time){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        sleep(time);
    }
    public void moveBack (double power, long time){
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        sleep(time);
    }
    public void moveRight (double power, long time){
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
        sleep(time);
    }
    public void moveLeft (double power, long time){
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
        sleep(time);
    }
    public void turnRight(double power, long time) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
        sleep(time);
    }
    public void turnLeft(double power, long time) {
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
        sleep(time);
    }
    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        ViperSlideRight.setPower(0);
    }
    public void intakeSpin(long time) {
        intake.setPower(0.25);
        sleep(time);
    }
    public void ViperUp(long time) {
        ViperSlideRight.setPower(-0.5);
        sleep(time);
    }

    public void ViperDown (long time) {
        ViperSlideRight.setPower(0.755);
    }
    public void clawOpen() {
        claw.setPosition(0.8);
    }
    public void clawClose() {
        claw.setPosition(2.5);
    }
    public void clawPivotScoring() {
        clawPivot.setPosition(0.3);
    }
    public void clawPivotNormal() {
        clawPivot.setPosition(0.05);
    }
    public void placePixel (long time) {
        intake.setPower(-0.25);
        sleep(time);
    }
}