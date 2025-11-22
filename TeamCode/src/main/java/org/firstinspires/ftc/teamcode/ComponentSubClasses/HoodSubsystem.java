package org.firstinspires.ftc.teamcode.ComponentSubClasses;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodSubsystem {
    private final Servo left;
    private final Servo right;

    public HoodSubsystem(HardwareMap hw, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        left = hw.get(Servo.class, "leftHoodServo");
        right = hw.get(Servo.class, "rightHoodServo");
        left.setDirection(Servo.Direction.REVERSE);
    }

    public void setAngle(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }
}

