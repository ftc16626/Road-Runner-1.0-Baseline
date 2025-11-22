package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "HoodActuatorTest", group = "robot")
@Disabled
public class HoodActuator extends LinearOpMode {
    //This is coded by a Rookie. It may be terrible
    public void runOpMode() {
        Servo rightHoodServo = null;
        Servo leftHoodServo = null;
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");

        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");

        leftHoodServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        leftHoodServo.setPosition(0);
        rightHoodServo.setPosition(0.0);
        while (opModeIsActive())

            if (gamepad2.dpad_up) {
                rightHoodServo.setPosition(0.425);
                leftHoodServo.setPosition(0.425);
            } else if (gamepad2.dpad_down) {
                rightHoodServo.setPosition(0.0);
                leftHoodServo.setPosition(0.0);
            }
        }
    }


