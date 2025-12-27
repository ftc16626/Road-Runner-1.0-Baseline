package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensors {

   NormalizedColorSensor colorSensor;

   public enum DetectedColor {
       GREEN,
       PURPLE,
       UNKNOWN

   }

   public void init(HardwareMap hwMap) {
       colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensorI, colorSensorII, colorSensorIII, colorSensorIV, colorSensorV,colorSensorVI");
   }

   public DetectedColor getDetectedColor(Telemetry telemetry){
       // Returns Red, Green, Blue, Alpha
       // Alpha is the brightness of the
       // color or how much light is returned back to sensor
       NormalizedRGBA colors = colorSensor.getNormalizedColors();

       // This will normalize the sensors
       // to the difference in light
       //depending on the distance from sensor
       float normRed, normGreen, normBlue;
       normRed = colors.red / colors.alpha;
       normGreen = colors.green / colors.alpha;
       normBlue = colors.blue / colors.alpha;

       telemetry.addData("red", normRed);
       telemetry.addData("green", normGreen);
       telemetry.addData("blue", normBlue);

       //COLOR VALUES FROM ARTFACTS
       /*
       GREEN = <.04, >.13, >.10
       PURPLE = <.06, <.07, >.09
        */

       if (normRed <0.04 && normGreen >0.06 && normBlue >0.04) {
           return DetectedColor.GREEN;
       } else if (normRed <0.08 && normGreen <0.09 && normBlue >0.08) {
           return DetectedColor.PURPLE;
       }
       else {

           return DetectedColor.UNKNOWN;
       }

   }

}
