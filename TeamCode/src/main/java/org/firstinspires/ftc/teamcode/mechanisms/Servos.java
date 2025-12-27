package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Servos {

    private Servo servoPos;
    private CRServo servoRot;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servo_pos");
        servoRot = hwMap.get(CRServo.class, "servo_rot");
        // can be used to set a range for all your servos
        // servoPos.scaleRange();
        // set your servo direction
        // servoPos.setDirection(Servo.Direction.REVERSE);

    }

    public void setServoPos(double angle){
        servoPos.setPosition(angle);
    }

    public void setServoRot (double power){
        servoRot.setPower(power);
    }
}
