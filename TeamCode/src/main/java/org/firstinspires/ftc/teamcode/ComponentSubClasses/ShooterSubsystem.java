package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Shooter handles spin-up, PID control wrapper, and firing sequences.
 * Also provides Actions compatible with Actions.runBlocking(...)
 */
public class ShooterSubsystem {
    private final Telemetry telemetry;
    private final HardwareMap hw;

    private DcMotorEx shooter;
    private Servo servoI, servoII, servoIII;

    // PID variables (from original)
    private double Kp = 0.4;
    private double Ki = 0.0; // your original used 0
    private double Kd = 0.1;

    private double Sum = 0.0;
    private double latestError = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Shooter properties
    private final double ticksPerRevolution = 28.0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hw = hardwareMap;
        this.telemetry = telemetry;

        shooter = hw.get(DcMotorEx.class, "shooter");
        servoI  = hw.get(Servo.class, "flipper1");
        servoII = hw.get(Servo.class, "flipper2");
        servoIII= hw.get(Servo.class, "flipper3");

        // ensure sensible starting positions
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);
    }

    /**
     * Blocking spin-up: attempts to reach target RPM or times out.
     * targetRPM in RPM, maxWaitSeconds is timeout
     */
    public void spinUpBlockingRPM(double targetRPM, double maxWaitSeconds) {
        double targetVelocity = (targetRPM / 60.0) * ticksPerRevolution;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        // try using velocity, fallback to power control
        try {
            shooter.setVelocity(targetVelocity);
        } catch (Exception e) {
            shooter.setPower(0.9);
        }
        // short wait
        while (t.seconds() < Math.min(0.6, maxWaitSeconds)) {
            // idle
        }
    }

    public void stop() {
        try {
            shooter.setPower(0);
        } catch (Exception ignored) {}
    }

    /**
     * Returns an Action that performs the complex shooter + servo sequence based on tag id.
     * This replicates your previous Action-based patterns.
     */
    public Action getFiringAction(final int artifactPattern) {
        return new Action() {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    // Start shooter closed-loop power via PID until velocity reached
                    initialized = true;
                    timer.reset();
                }

                // run firing sequence for up to 6 seconds (original used many waits)
                if (timer.seconds() > 6.0) {
                    // ensure servos reset
                    servoI.setPosition(0.5);
                    servoII.setPosition(0.5);
                    servoIII.setPosition(0.51);
                    stop();
                    return false;
                }

                // We'll mimic your original conditional sequences but more linear and safer.
                // Use helper to run each pattern with timed servo moves
                runPatternOnce(artifactPattern);
                return false; // action finished immediately after performing pattern
            }

            private void runPatternOnce(int pattern) {
                // Spin shooter to approximate target while firing pattern
                double targetRPM = 2150;
                double targetVelocity = (targetRPM / 60.0) * ticksPerRevolution;

                // naive loop to keep power while running servo steps
                ElapsedTime seqTimer = new ElapsedTime();
                seqTimer.reset();

                // pattern mapping (preserve your original servo orders)
                if (pattern == 21) {
                    // Sequence for 21
                    shooter.setPower(pidOutputFor(targetVelocity));
                    servoII.setPosition(0.9);
                    sleepMillis(1000);
                    servoI.setPosition(0.47);
                    sleepMillis(1000);
                    servoI.setPosition(0.9);
                    sleepMillis(750);
                    servoIII.setPosition(0.53);
                    sleepMillis(750);
                    servoIII.setPosition(0.1);
                } else if (pattern == 22) {
                    shooter.setPower(pidOutputFor(targetVelocity));
                    servoI.setPosition(0.9);
                    sleepMillis(1000);
                    servoII.setPosition(0.47);
                    sleepMillis(1000);
                    servoII.setPosition(0.9);
                    sleepMillis(750);
                    servoIII.setPosition(0.51);
                    sleepMillis(750);
                    servoIII.setPosition(0.1);
                } else if (pattern == 23) {
                    shooter.setPower(pidOutputFor(targetVelocity));
                    servoI.setPosition(0.9);
                    sleepMillis(1000);
                    servoIII.setPosition(0.51);
                    sleepMillis(1000);
                    servoIII.setPosition(0.1);
                    sleepMillis(750);
                    servoII.setPosition(0.47);
                    sleepMillis(750);
                    servoII.setPosition(0.9);
                } else {
                    // default
                    shooter.setPower(pidOutputFor(targetVelocity));
                    servoII.setPosition(0.9);
                    sleepMillis(1000);
                    servoI.setPosition(0.47);
                    sleepMillis(1000);
                    servoI.setPosition(0.9);
                    sleepMillis(750);
                    servoIII.setPosition(0.53);
                    sleepMillis(750);
                    servoIII.setPosition(0.1);
                }

                // clean up
                shooter.setPower(0);
                servoI.setPosition(0.5);
                servoII.setPosition(0.5);
                servoIII.setPosition(0.51);
            }
        };
    }

    /**
     * Returns a small Action used in the auto's earlier call of shooter.Scan() in original.
     * This one is a no-op for compatibility and can be removed if not used.
     */
    public Action getScanAction() {
        return new Action() {
            private boolean initialized = false;
            private final ElapsedTime t = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    t.reset();
                }
                return t.seconds() < 0.5;
            }
        };
    }

    // ----- PID helper (simpler and safer)
    private double pidOutputFor(double targetVelocity) {
        double currentVelocity;
        try {
            currentVelocity = shooter.getVelocity();
        } catch (Exception e) {
            currentVelocity = 0;
        }

        double dt = pidTimer.seconds();
        if (dt <= 0) dt = 1e-6;
        pidTimer.reset();

        double error = targetVelocity - currentVelocity;
        Sum += error * dt;
        double derivative = (error - latestError) / dt;
        latestError = error;

        double output = (Kp * error) + (Ki * Sum) + (Kd * derivative);

        // clamp output to reasonable range for setPower fallback if needed
        if (output > 1.0) output = 1.0;
        if (output < -1.0) output = -1.0;
        return output;
    }

    private void sleepMillis(long ms) {
        long target = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < target) {
            // busy-wait small amount (OpMode will be calling idle externally)
        }
    }
}
