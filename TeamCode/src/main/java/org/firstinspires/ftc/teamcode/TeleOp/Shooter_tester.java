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
        double areaOnex = 1.30;
        double areaOneAngle = Math.toRadians(79.967);
        double areaTwox = 0.675;
        double areaTwoAngle = Math.toRadians(71.184);
        double initialVelocity = 0; // Example initial velocity (m/s)
        double deltaXOne = areaOnex;          // Example horizontal distance (m)
        double theta = areaOneAngle; // Example launch angle (45 degrees)
        //double calculatedDeltaY = (deltaX * Math.tan(theta)) - (GRAVITY * Math.pow(deltaX, 2)) / (2 * Math.pow(initialVelocity * Math.cos(theta), 2));
        //telemetry.addData("Calculated Trajectory", calculatedDeltaY);
        telemetry.update();
        sleep( 2000);
        double DeltaXTwo = areaTwox;    // Target horizontal distance (m)
        double targetDeltaY = 0.23;    // Target vertical distance (m)
        double launchAngle = Math.toRadians(30); // Target launch angle (30 degrees)
        // run until the end of the match (driver presses STOP)
        double numerator = GRAVITY * Math.pow(deltaXOne, 2);
        double denominator = 2 * Math.pow(Math.cos(areaTwoAngle), 2) * (deltaXOne * Math.tan(areaTwoAngle) - targetDeltaY);
        double drivingSpeed = GRAVITY * Math.pow(deltaXOne,2) / 2 * (Math.pow(Math.cos(theta), 2)) * (deltaXOne * Math.tan(theta) - targetDeltaY);
        double requiredDrivingSpeed = Math.sqrt(drivingSpeed);
        double wheelRadius = 2;
        double omega = requiredDrivingSpeed / wheelRadius;
        double requiredOmega = omega * 30 / Math.PI;

        if (denominator <= 0) {
            telemetry.addData("Error", "No valid initial velocity found for these parameters.");
            telemetry.update();
            return;
        }
        double initialVelocitySquared = numerator / denominator;
        double requiredInitialVelocity = Math.sqrt(initialVelocitySquared);
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


            NormalizedRGBA Cola1 = first.getNormalizedColors();
            Color.colorToHSV(Cola1.toColor(), hsvValues);

            if (gamepad2.triangle) {

                flipper1.setPosition(-.1);
              //  if (Cola1.green > Cola1.blue) {
                //    flipper1.setPosition(-0.1);
                  //  sleep(2000);
                  //  flipper1.setPosition(0.1);
                //}

                telemetry.addLine()
                        .addData("Red", "%.3f", Cola1.red)
                        .addData("Green", "%.3f", Cola1.green)
                        .addData("Blue", "%.3f", Cola1.blue);
                telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValues[0])
                        .addData("Saturation", "%.3f", hsvValues[1])
                        .addData("Value", "%.3f", hsvValues[2]);
                telemetry.addData("Alpha", "%.3f", Cola1.alpha);
                //      flipper2.setPosition(25);
                //      flipper3.setPosition(25);



            }
            loop();
            if (gamepad2.circle){
                shooter.setPower(requiredOmega);
            }
            else {
                shooter.setPower(0);
            }


        }}



            }
