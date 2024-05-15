package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "auto_test", group = "Autonomous")
public class auto_test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Lift.Claw claw = new Lift.Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut;

        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3)
                .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3)
                .build();
        trajectoryAction3 = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3)
                .build();
        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryAction1;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryAction2;
        } else {
            trajectoryActionChosen = trajectoryAction3;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
    }

    public static class Lift implements org.firstinspires.ftc.teamcode.Lift {
        private DcMotorEx rightViper;
        private DcMotorEx leftViper;

        public Lift(HardwareMap hardwareMap) {
            rightViper = hardwareMap.get(DcMotorEx.class, "rightViper");
            leftViper = hardwareMap.get(DcMotorEx.class, "leftViper");
            rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightViper.setDirection(DcMotorSimple.Direction.FORWARD);
            leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftViper.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightViper.setPower(-1);
                    leftViper.setPower(1);
                    initialized = true;
                }
                return false;
            }

            public Action liftUp() {
                return new LiftUp();
            }

            public class LiftDown implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        rightViper.setPower(1);
                        leftViper.setPower(-1);
                        initialized = true;
                    }
                    return false;
                }
            }

            public Action liftDown() {
                return new LiftDown();
            }
        }

        public static class Claw {
            private Servo claw;

            public Claw(HardwareMap hardwareMap) {
                claw = hardwareMap.get(Servo.class, "claw");
            }

            public class CloseClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(0.5);
                    return false;
                }
            }

            public Action closeClaw() {
                return new CloseClaw();
            }

            public class OpenClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(0);
                    return false;
                }
            }

            public Action openClaw() {
                return new OpenClaw();
            }
        }

        @Override
        public void runOpMode() {

        }
    }
}