package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BotServos {
    Servo ServoI;
    Servo ServoII;
    Servo ServoIII;
    CRServo IntakeServo;
    public void getHardwareMap(HardwareMap hardwareMap){
        ServoI = hardwareMap.get(Servo.class, "flipper1");
        ServoII = hardwareMap.get(Servo.class, "flipper2");
        ServoIII = hardwareMap.get(Servo.class, "flipper3");

    }
}
