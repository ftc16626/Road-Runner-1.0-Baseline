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

    public HoodSubsystemOdometryReady(HardwareMap hardwareMap) {
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHoodServo.setDirection(Servo.Direction.REVERSE);
        rightHoodServo.setDirection(Servo.Direction.FORWARD);

        flipper1 = hardwareMap.get(Servo.class, "flipper1");
        flipper2 = hardwareMap.get(Servo.class, "flipper2");
        flipper3 = hardwareMap.get(Servo.class, "flipper3");
    }

    public void controlFlippers(Gamepad gamepad) {
        // Hood RPM/position selection
        if (gamepad.dpad_left) {
            setHood(hoodPosClose);
        } else if (gamepad.dpad_right) {
            setHood(hoodPosFar);
        }

        // Quick flipper pulses
        if (gamepad.circle) pulse(flipper3, 0.1, 0.53);
        if (gamepad.a) pulse(flipper2, 0.9, 0.47);
        if (gamepad.square) pulse(flipper1, 0.9, 0.47);

        // Set default positions
        if (gamepad.triangle) {
            flipper1.setPosition(0.53);
            flipper2.setPosition(0.53);
            flipper3.setPosition(0.49);
        }
        if (gamepad.dpad_down) {
            flipper1.setPosition(0.47);
            flipper2.setPosition(0.47);
            flipper3.setPosition(0.53);
        }
    }

    private void setHood(double pos) {
        leftHoodServo.setPosition(pos);
        rightHoodServo.setPosition(pos);
    }

    private void pulse(Servo s, double a, double b) {
        s.setPosition(a);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        s.setPosition(b);
    }

    // -------------------- Odometry placeholders --------------------
    // Future integration: servo angle can be read for precise shot targeting.
    // double getHoodAngle() { return ???; }
}

