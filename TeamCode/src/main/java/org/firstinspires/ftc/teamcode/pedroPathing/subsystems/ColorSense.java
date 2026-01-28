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
    PanelsTelemetry dashboard = PanelsTelemetry.INSTANCE;
    TelemetryManager dashboardTelemetry = dashboard.getTelemetry();
    private double hue;
    public ColorSense(HardwareMap hardwareMap){
        colorSense = hardwareMap.get(NormalizedColorSensor.class, "colorSense");
    }
    public boolean colorIsDetected(){
        if(hue != 0){
            return true;
        }
        return false;
    }

    public void update(){
        dashboardTelemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSense).getLightDetected());
        NormalizedRGBA colors = colorSense.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        //Determining the amount of red, green, and blue

        //Determining HSV and alpha
        dashboardTelemetry.addData("Hue", hue);
        dashboardTelemetry.addData("Alpha", colors.alpha);
        if(JavaUtil.colorToSaturation(colors.toColor()) == 0){
            dashboardTelemetry.addData("Detected", "Nothing");
        }else if(hue >= 150 && hue <= 180){
            dashboardTelemetry.addData("Detected", "Green");
        }else if(hue >= 210 && hue <= 240){
            dashboardTelemetry.addData("Detected", "Purple");
        }else{
            dashboardTelemetry.addData("Detected", "Other");
        }
    }
}
