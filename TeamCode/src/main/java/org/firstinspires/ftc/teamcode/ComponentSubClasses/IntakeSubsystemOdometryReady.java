package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Intake subsystem: controls roller servo for ball intake.
 * Odometry-ready: can add encoder feedback for counting rotations or distance.
 */
public class IntakeSubsystemOdometryReady {

    public CRServo rollerServo;

    public IntakeSubsystemOdometryReady(HardwareMap hardwareMap) {
        rollerServo = hardwareMap.get(CRServo.class, "roller");
    }

    public void controlIntake(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            rollerServo.setPower(1.0);
        } else if (gamepad.left_bumper) {
            rollerServo.setPower(-1.0);
        } else {
            rollerServo.setPower(0.0);
        }
    }

    // -------------------- Odometry placeholders --------------------
    // If roller encoder is available, can track number of balls collected
    // int getRollerTicks() { return rollerEncoder.getCurrentPosition(); }
}

