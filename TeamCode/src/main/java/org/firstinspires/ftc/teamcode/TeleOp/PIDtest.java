package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.tuning.PIDFController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.tuning.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous(name = "PIDtest", group = "robot")

public class PIDtest extends LinearOpMode {
 //   private DcMotorEx shooter1;
    private DcMotorEx shooter2;
  //  private DcMotorEx shooter3;
    public static double Kp = 0.4;
    public static double Ki = 0;
    public static double Kd = 0.1;

    public static double targetRPM = 3000;
    public static double ticksPerRevolution = 28;
    public double targetVelocity = 2300;
    private final double TARGET_VELOCITY_TICKS_PER_SECOND = 1000;
    public double latestError = 0;
    public double Sum2 = 0;
   // public double currentVelocity1;
    public double currentVelocity2;
   // public double currentVelocity3;
    ElapsedTime timer = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public void runOpMode() {
      //  shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
     //   shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
      //  shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  shooter3 = hardwareMap.get(DcMotorEx.class, "shooter");
      //  shooter3.setDirection(DcMotorSimple.Direction.FORWARD);
      //  shooter3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
  //      shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   //     shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //    shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  shooter3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  shooter3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //  shooter3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()){
        //    double currentVelocity = shooter1.getVelocity();
            double power = PIDControl(targetVelocity, currentVelocity2);
            packet.put("Target Velocity", targetVelocity);
            packet.put("Current Velocity", currentVelocity2 );
            packet.put("error", latestError);
            shooter2.setPower(power);
            dashboard.sendTelemetryPacket(packet);

        }
    }


    public double PIDControl (double reference, double state){
     //   currentVelocity1 = shooter1.getVelocity();
        currentVelocity2 = shooter2.getVelocity();
     //   currentVelocity3 = shooter3.getVelocity();
        double deltaTime = timer.seconds();
        timer.reset();

     //   double error1 = targetVelocity - currentVelocity1;
        double error2 = targetVelocity - currentVelocity2;
    //    double error3 = targetVelocity - currentVelocity3;
       // Sum1 += error1 * deltaTime;
        Sum2 += error2 * deltaTime;
    //    Sum3 += error3 * deltaTime;
        latestError = error2;
        double derivative = (error2 - latestError) / timer.seconds();
        timer.reset();

        double output = (error2 * Kp) + (derivative + Kd) + (Sum2 * Ki);
        return output;
    }

}
