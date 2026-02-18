package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class ColorSense {
    private NormalizedColorSensor colorSense;
    private NormalizedColorSensor colorSense2;
    TelemetryManager dashboardTelemetry;
    private double hue;
    private double hue2;
    private int color;
    public ColorSense(HardwareMap hardwareMap, TelemetryManager dashboard){
        colorSense = hardwareMap.get(NormalizedColorSensor.class, "colorSense");
        colorSense2 = hardwareMap.get(NormalizedColorSensor.class, "colorSense2");
        dashboardTelemetry = dashboard;
    }
    public boolean colorIsDetected(){
        if(hue != 0){
            return true;
        }
        return false;
    }

    public boolean ballDetected(){
        return color == 1 || color == 2;
    }

    public int getColor(){
        return color;
    }

    public void update(){
        dashboardTelemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSense).getLightDetected());
        NormalizedRGBA colors = colorSense.getNormalizedColors();
        NormalizedRGBA colors2 = colorSense2.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());
        hue2 = JavaUtil.colorToHue(colors2.toColor());

        //Determining the amount of red, green, and blue

        //Determining HSV and alpha
        dashboardTelemetry.addData("Hue", hue);
        dashboardTelemetry.addData("Hue2", hue2);
        dashboardTelemetry.addData("Alpha", colors.alpha);
        dashboardTelemetry.addData("Alpha2", colors2.alpha);

        if(((colors.alpha < 0.99 && colors.alpha > 0.8) && (hue >= 150 && hue <= 175)) || ((hue2 >= 150 && hue2 <= 175) && (colors2.alpha < 0.99 && colors2.alpha > 0.8))){
            dashboardTelemetry.addData("Detected", "Green");
            color = 2;
        }else if(((colors.alpha < 0.99 && colors.alpha > 0.8) && (hue >= 215 && hue <= 230)) || ((hue2 >= 215 && hue2 <= 230) && (colors2.alpha < 0.99 && colors2.alpha > 0.8))){
            dashboardTelemetry.addData("Detected", "Purple");
            color = 1;
        }else if(colors.alpha < 0.1){
            dashboardTelemetry.addData("Detected", "Nothing");
            color = 0;
        }else{
            dashboardTelemetry.addData("Detected", "Other");
            color = 0;
        }
    }
}
