package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Hood subsystem for shooter angle control and flipper servos.
 * Odometry-ready: can integrate positional feedback if needed.
 */
public class HoodSubsystemOdometryReady {

    public Servo leftHoodServo, rightHoodServo;
    public Servo flipper1, flipper2, flipper3;


    private double hoodPosClose = 0.225;
    private double hoodPosFar = 0.40;
    private double hoodPosGoal = 0.025;
    public double curTargetVelocity = 1213.333333;

    public HoodSubsystemOdometryReady(HardwareMap hardwareMap) {

        flipper1 = hardwareMap.get(Servo.class, "flipper1");
        flipper2 = hardwareMap.get(Servo.class, "flipper2");
        flipper3 = hardwareMap.get(Servo.class, "flipper3");

    }
    private long pulseEndTime = 0;
    private Servo pulsingServo = null;
    private double pulseReturnPos = 0;

    public void updateFlipperPulse() {
        if (pulsingServo != null && System.currentTimeMillis() >= pulseEndTime) {
            pulsingServo.setPosition(pulseReturnPos);
            pulsingServo = null;
        }
    }

    private void startPulse(Servo s, double out, double back, long durationMs) {
        if (pulsingServo == null) { // prevent stacking pulses
            s.setPosition(out);
            pulsingServo = s;
            pulseReturnPos = back;
            pulseEndTime = System.currentTimeMillis() + durationMs;
        }
    }

    public void controlFlippers(Gamepad gamepad) {
        // Hood RPM/position selection


        // Quick flipper pulses
        if (gamepad.circle) startPulse(flipper3, 0.1, 0.53, 100);
        if (gamepad.a)      startPulse(flipper2, 0.9, 0.47, 100);
        if (gamepad.square) startPulse(flipper1, 0.9, 0.47, 100);

        updateFlipperPulse();

        // Set default positions
        if (gamepad.triangle) {
            flipper1.setPosition(0.53);
            flipper2.setPosition(0.53);
            flipper3.setPosition(0.49);
        }
        if (gamepad.dpad_down) {
            flipper1.setPosition(0.47);
            flipper2.setPosition(0.47);
            flipper3.setPosition(0.55);
            flipper3.setPosition(0.55);
        }
    }

    private void setHood(double pos) {
        leftHoodServo.setPosition(pos);
        rightHoodServo.setPosition(pos);
    }



    // -------------------- Odometry placeholders --------------------
    // Future integration: servo angle can be read for precise shot targeting.
    // double getHoodAngle() { return ???; }
}

