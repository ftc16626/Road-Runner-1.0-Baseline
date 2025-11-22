package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    private final CRServo intake;

    public IntakeSubsystem(HardwareMap hw, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        intake = hw.get(CRServo.class, "roller");
    }

    public void forward() {
        intake.setPower(1.0);
    }

    public void reverse() {
        intake.setPower(-1.0);
    }

    public void stop() {
        intake.setPower(0);
    }
}
