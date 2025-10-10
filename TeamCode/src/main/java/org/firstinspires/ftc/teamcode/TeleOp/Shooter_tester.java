package org.firstinspires.ftc.teamcode.TeleOp;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp  (name = "Shooter Tester", group = "robot")
public class Shooter_tester extends LinearOpMode{
   // private DcMotor shooter;
    private Servo flipper1;
    private DcMotor shooter;
  //  private Servo flipper2;
 //   private Servo flipper3;
    private NormalizedColorSensor first;
    private NormalizedColorSensor second;
    private NormalizedColorSensor third;
    private NormalizedColorSensor fourth;
    private NormalizedColorSensor fifth;
    private NormalizedColorSensor sixth;
    View relativeLayout;
    public void runOpMode() {
        final double GRAVITY = 9.81;
        float gain = 2;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        if (first instanceof SwitchableLight) {
            ((SwitchableLight) first).enableLight(true);
        }
        if (second instanceof SwitchableLight) {
            ((SwitchableLight) second).enableLight(true);
        }

        if (third instanceof SwitchableLight) {
            ((SwitchableLight) third).enableLight(true);
        }
        if (fourth instanceof SwitchableLight) {
            ((SwitchableLight) fourth).enableLight(true);
        }
        if (fifth instanceof SwitchableLight) {
            ((SwitchableLight) fifth).enableLight(true);
        }

        if (sixth instanceof SwitchableLight) {
            ((SwitchableLight) sixth).enableLight(true);
        }
        final float[] hsvValues = new float[3];
     //   shooter = hardwareMap.get(DcMotor.class, "shooter");
        first = hardwareMap.get(NormalizedColorSensor.class, "first");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        flipper1 = hardwareMap.get(Servo.class, "flipper1");
        //flipper2 = hardwareMap.get(Servo.class, "flipper2");
        //flipper3 = hardwareMap.get(Servo.class, "flipper3");
        first.setGain(gain);

      //  shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad2.right_bumper){
                shooter.setPower(1);
            }

            NormalizedRGBA Cola1 = first.getNormalizedColors();
            Color.colorToHSV(Cola1.toColor(), hsvValues);

            if (gamepad2.triangle) {
               flipper1.setPosition(0.25);
                if (Cola1.blue < 0.20) {
                    sleep(2000);
                    flipper1.setPosition(1);
                    sleep(2000);
                    flipper1.setPosition(0);
                }

                //      flipper2.setPosition(25);
                //      flipper3.setPosition(25);

                shooter.setPower(1);

            }

        }}



            }
