package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
        (name = "z Tool PIDF tuning", group = "z")
public class PIDF extends OpMode {

    // wheel 1 P = 80 F = 13.9
    // wheel 2 P = 60 F = 12.4
    // wheel 3 P = 60 F = 12.045
    public DcMotorEx wheel1;
    public double FarVelocity = 2300;
    public  double CloseVelocity = 1250;
    double curTargetVelocity = FarVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;



    @Override
    public void init(){
        wheel1= hardwareMap.get(DcMotorEx.class, "shooter3");
        wheel1.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        wheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        wheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
        wheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        wheel1.setVelocity(curTargetVelocity);
        double curVelocity = wheel1.getVelocity();
        double error = curTargetVelocity - curVelocity;
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("TuningP", "%.4f (D-Pad U/D)",P );
        telemetry.addData("TuningF", "%.4f (D-Pad L/R)",F );
        telemetry.addData("Step Size", "%.4f (Circle Button)", stepSizes[stepIndex]);

    }
}
