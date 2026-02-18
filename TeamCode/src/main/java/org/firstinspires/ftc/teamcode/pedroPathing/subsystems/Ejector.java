package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Ejector {
    public static double ejectorPos = .72;
    public static double defaultEjectPos = .72;
    public static double kickPos = .99;
    Servo ejector;
    ElapsedTime quickfireTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static int quickfireTime = 400;
    public Ejector(HardwareMap hardwareMap){
        ejector = hardwareMap.get(Servo.class, "ejector");
    }

    public void ejectorTo(double p){
        ejectorPos = p;
    }

    public void quickfire(){
        ejector.setPosition(kickPos);
        quickfireTimer.reset();
        while(quickfireTimer.milliseconds() < quickfireTime){

        }
        ejector.setPosition(defaultEjectPos);
    }

    public void update(){
        ejector.setPosition(ejectorPos);
    }
}
