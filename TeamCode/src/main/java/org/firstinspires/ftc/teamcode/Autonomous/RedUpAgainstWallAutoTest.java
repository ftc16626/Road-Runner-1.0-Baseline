package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ComponentSubClasses.DriveSubsystem;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.HoodSubsystem;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.VisionSubsystem;

/**
 * Orchestrator opmode. Keeps sequencing short and delegates behavior to subsystems.
 */
@Autonomous(name="TheRedUpAgainstWallAutoYouShouldUseTest", group="Robot")
public class RedUpAgainstWallAutoTest extends LinearOpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private VisionSubsystem vision;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;

    @Override
    public void runOpMode() {

        // instantiate subsystems (each registers their hardware)
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap, telemetry);

        // init vision (camera start)
        vision.init();

        telemetry.addLine("Initialized subsystems");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // Quick scan (blocking) - returns tag id or -1
        int tag = vision.scanOnceBlocking();

        // Sequence: scan -> position -> spin up and fire sequences like original
        // Use RoadRunner Actions that wrap subsystem methods where appropriate.
        // We're preserving your original order and behaviors.

        // Example using Actions.runBlocking with shooter actions (ShooterSubsystem provides Action objects)
        Actions.runBlocking(new SequentialAction(shooter.getScanAction())); // duplicates original scan if you'd prefer to rely on vision.scanOnceBlocking remove this

        // move into scan area (roughly matches your previous steps)
        drive.encoderDriveBlocking(0.4, -2, -2, -2, -2, 1.0, false, 0, 0.0); // small one-frame adjustments
        drive.encoderDriveBlocking(0.4, -5, -5, -5, -5, 1.2, false, 0, 0.0);
        drive.encoderDriveBlocking(0.4, -5, -5, -5, -5, 1.2, false, 0, 0.0);

        // spin up shooter (brief blocking)
        shooter.spinUpBlockingRPM(2150, 0.6); // 0.6s max wait as in original

        // move into shooting position
        drive.encoderDriveBlocking(0.4, 2, -2, 2, -2, 0.6, false, 0, 0.0);
        drive.encoderDriveBlocking(0.4, 1, 1, 1, 1, 0.4, false, 0, 0.0);

        // Run firing sequence (non-blocking style wrapped in a blocking call to finish)
        Actions.runBlocking(new SequentialAction(shooter.getFiringAction(tag)));

        // Exit shooting area
        drive.encoderDriveBlocking(0.5, 4, 4, 4, 4, 1.0, false, 0, 0.0);

        shooter.stop();
        intake.stop();
        hood.setAngle(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(500);
    }
}
