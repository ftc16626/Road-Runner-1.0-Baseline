package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Physics Trajectory Calculation", group = "robot")
public class PhysicsTrajectoryCalculation extends LinearOpMode {

    // Define gravity constant (in meters per second squared)
    private static final double GRAVITY = 9.81;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // --- Calculation 1: Calculate the final vertical position (deltaY) given an initial velocity ---
        // This simulates where the projectile will land
        double initialVelocity = 5.0; // Example initial velocity (m/s)
        double deltaX = 1.0;          // Example horizontal distance (m)
        double theta = Math.toRadians(45); // Example launch angle (45 degrees)

        // Calculate the resulting trajectory (deltaY)
        double calculatedDeltaY = (deltaX * Math.tan(theta)) - (GRAVITY * Math.pow(deltaX, 2)) / (2 * Math.pow(initialVelocity * Math.cos(theta), 2));

        telemetry.addData("Calculated Trajectory", calculatedDeltaY);
        telemetry.update();
        sleep(2000); // Wait for telemetry to display

        // --- Calculation 2: Find the initial velocity required to hit a specific target (deltaX, deltaY) ---
        // This is a more common use case for FTC, where you need to adjust motor power
        // to hit a target.

        // Define your known target
        double targetDeltaX = 2.0;    // Target horizontal distance (m)
        double targetDeltaY = 0.5;    // Target vertical distance (m)
        double launchAngle = Math.toRadians(30); // Target launch angle (30 degrees)

        // Rearrange the trajectory equation to solve for initial velocity squared (v0^2)
        // v0^2 = (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - y))
        double numerator = GRAVITY * Math.pow(targetDeltaX, 2);
        double denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * (targetDeltaX * Math.tan(launchAngle) - targetDeltaY);

        // Check for invalid inputs (e.g., target is too high)
        if (denominator <= 0) {
            telemetry.addData("Error", "No valid initial velocity found for these parameters.");
            telemetry.update();
            return;
        }

        double initialVelocitySquared = numerator / denominator;
        double requiredInitialVelocity = Math.sqrt(initialVelocitySquared);

        telemetry.addData("Initial Velocity^2 (required)", initialVelocitySquared);
        telemetry.addData("Initial Velocity (required)", requiredInitialVelocity);
        telemetry.update();

        sleep(10000); // Wait for telemetry to display before ending OpMode
    }
}