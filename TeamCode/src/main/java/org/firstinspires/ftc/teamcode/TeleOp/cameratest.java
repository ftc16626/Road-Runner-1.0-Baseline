package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="cameratest", group = "robot")
@Disabled
public class cameratest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initAprilTag();

        telemetry.addData("DS preview on", "EasyOpenCV");
        telemetry.addData("Camera preview on", "Webcam");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // Process detections
            if (!currentDetections.isEmpty()) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addData("ID", detection.id);
                        telemetry.addData("XYZ", detection.ftcPose.x + ", " + detection.ftcPose.y + ", " + detection.ftcPose.z);
                        telemetry.addData("Rotation", detection.ftcPose.roll + ", " + detection.ftcPose.pitch + ", " + detection.ftcPose.yaw);
                    }
                }
            } else {
                telemetry.addData("AprilTag", "Not detected");
            }
            telemetry.update();

            sleep(20); // Optional: reduce CPU usage
        }

        visionPortal.close(); // Close the vision portal when done
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Use your webcam's configured name
                .addProcessor(aprilTag)
                .build();
    }
}


