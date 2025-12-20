package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF extends OpMode {

    // wheel 1 pid sucks wheel is too tight cant over compensate yes
    // wheel 2 P = 60 F = 12.4
    // wheel 3 P = 60 F = 15.45
    public DcMotorEx wheel2;
    public double FarVelocity = 2300;
    public  double CloseVelocity = 1250;
    double curTargetVelocity = FarVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;



    @Override
    public void init(){
        wheel2= hardwareMap.get(DcMotorEx.class, "shooter2");
        wheel2.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        wheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        wheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("int complete");
    }
    @Override
    public void loop(){
        if (gamepad1.triangleWasPressed()){
            if (curTargetVelocity == FarVelocity){
                curTargetVelocity = CloseVelocity;
            }
            else {
                curTargetVelocity =  FarVelocity;
            }
        }
        if (gamepad1.circleWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes [stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes [stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        wheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        wheel2.setVelocity(curTargetVelocity);
        double curVelocity = wheel2.getVelocity();
        double error = curTargetVelocity - curVelocity;
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("TuningP", "%.4f (D-Pad U/D)",P );
        telemetry.addData("TuningF", "%.4f (D-Pad L/R)",F );
        telemetry.addData("Step Size", "%.4f (Circle Button)", stepSizes[stepIndex]);

    }
}
