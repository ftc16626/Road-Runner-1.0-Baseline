package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * VisionSubsystemOdometryReady
 * Handles AprilTag detection and stores the last detected ID for use in autonomous.
 * Designed to be odometry-ready in the future (can integrate pose/coordinate info later).
 */
public class VisionSubsystemOdometryReady {

    private HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Last detected AprilTag ID
    private int lastDetectedTagID = -1;

    // Constructor
    public VisionSubsystemOdometryReady(HardwareMap hw) {
        this.hardwareMap = hw;
    }

    /**
     * Initialize AprilTag processor and VisionPortal
     */
    public void initAprilTag() {
        // Build AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        // Build VisionPortal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Call periodically in autonomous to update the last seen AprilTag ID
     */
    public void updateDetection() {
        if (aprilTag != null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                // For simplicity, just take the first detected tag
                lastDetectedTagID = detections.get(0).id;
            }
        }
    }

    /**
     * Returns the last detected AprilTag ID
     * @return -1 if no tag detected yet
     */
    public int getLastDetectedTagID() {
        return lastDetectedTagID;
    }

    /**
     * Accessor for the VisionPortal (for advanced usage)
     */
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    /**
     * Accessor for the AprilTagProcessor
     */
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }
}
