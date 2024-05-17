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
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class auto_test extends LinearOpMode {

    public class Claw {
        private Servo claw;
        private Servo claw_wrist;
        private Servo claw_elbow;
        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
            claw_wrist = hardwareMap.get(Servo.class, "claw_wrist");
            claw_elbow = hardwareMap.get(Servo.class, "claw_elbow");
        }


        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                sleep(250);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.5);
                sleep(250);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
        public class TurnClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_wrist.setPosition(0.35);
                sleep(250);
                return false;
            }
        }
        public Action turnClaw() {
            return new TurnClaw();
        }
        public class WristNorm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_wrist.setPosition(0);
                sleep(250);
                return false;
            }
        }
        public Action wristNorm() {
            return new WristNorm();
        }
        public class ElbowOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_elbow.setPosition(0.45);
                sleep(250);
                return false;
            }
        }
        public Action elbowOut() {
            return new ElbowOut();
        }
        public class ElbowBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw_elbow.setPosition(0.225);
                sleep(250);
                return false;
            }
        }
        public Action elbowBack() {
            return new ElbowBack();
        }
    }
    public class IntakeDrop {
        public Servo intake_drop;

        public IntakeDrop(HardwareMap hardwareMap) {
            intake_drop = hardwareMap.get(Servo.class, "intake_drop");
        }
        public class PullUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_drop.setPosition(0);
                sleep(250);
                return false;
            }
        }
        public Action pullUp() {
            return new PullUp();
        }
        public class Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake_drop.setPosition(0.9);
                sleep(250);
                return false;
            }
        }

        public Action dropIntake() {
            return new Drop();
        }


    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Claw claw = new Claw(hardwareMap);
        IntakeDrop intake_drop = new IntakeDrop(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToY(40)
                .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(-90))
                .lineToX(6)
                .build();
        trajectoryAction3 = drive.actionBuilder(drive.pose)
                //
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
        Action trajectoryActionChosen2;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryAction1;
            trajectoryActionChosen2 = trajectoryAction2;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryAction1;
            trajectoryActionChosen2 = trajectoryAction2;
        } else {
            trajectoryActionChosen = trajectoryAction1;
            trajectoryActionChosen2 = trajectoryAction2;
        }

        Actions.runBlocking(
                new SequentialAction(
                        intake_drop.dropIntake(),
                        claw.closeClaw(),
                        claw.wristNorm(),
                        claw.elbowBack(),
                        trajectoryActionChosen,
                        intake_drop.pullUp(),
                        trajectoryActionChosen2,
                        claw.elbowOut(),
                        claw.turnClaw(),
                        claw.openClaw(),
                        claw.wristNorm(),
                        claw.elbowBack(),
                        trajectoryAction3
                )
        );
    }
}