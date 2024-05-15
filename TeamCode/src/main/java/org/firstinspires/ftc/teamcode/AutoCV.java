package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.jar.Attributes;

@Autonomous
public class AutoCV extends LinearOpMode {
    /*
     * If program has a build folder error try clearing the build
     */
    OpenCvWebcam camera;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToY(70)
                .build();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        OpenCVDetection detector = new OpenCVDetection(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set |
        /*
         * Below is an example of a lambda expression which is in simply an anonymous function.
         * Since we are only executing one statement we are able to remove the curly braces and semicolon
         * making it look much cleaner.
         * Note that this is a feature strictly for SDK 8+, if Java 7 is being used use this code instead.
         * To change preferences press command and ; to open up preference window.
         *
         * Lambda Expression *
         * camera.openCameraDeviceAsync(() -> camera.startStreaming (320,240, OpenCvCameraRotation. UPRIGHT));
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();
        if (isStopRequested()) return;

        Action trajectoryActionChosen = null;
        switch (detector.getLocation()) {
            case Left:
                // ...
                trajectoryActionChosen = trajectoryAction1;
                break;
            case Right:
                // ...
                break;
            case Middle:
                // ...
                break;
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
        camera.stopStreaming();
    }
}