package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Simple vision wrapper for AprilTag scanning.
 */
public class VisionSubsystem {
    private final HardwareMap hw;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;

    public VisionSubsystem(HardwareMap hw, Telemetry telemetry) {
        this.hw = hw;
        this.telemetry = telemetry;
    }

    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "allSeeingEye"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Performs a short blocking scan and returns detected tag id (21,22,23) or -1
     */
    public int scanOnceBlocking() {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        // give camera a short time to produce detections
        while (t.seconds() < 0.25) { /* wait briefly */ }

        List<AprilTagDetection> dets = aprilTag.getDetections();
        int artifactPattern = -1;
        if (dets != null && !dets.isEmpty()) {
            for (AprilTagDetection d : dets) {
                if (d.id == 21 || d.id == 22 || d.id == 23) {
                    artifactPattern = d.id;
                    telemetry.addData("Tag", d.id);
                }
            }
        } else {
            telemetry.addData("AprilTag", "Not seen");
        }
        telemetry.update();
        try { portal.close(); } catch (Exception ignored) {}
        return artifactPattern;
    }
}


