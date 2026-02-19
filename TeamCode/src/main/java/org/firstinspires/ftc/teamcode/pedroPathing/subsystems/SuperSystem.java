package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class SuperSystem {
    Index index;
    Ejector ejector;
    Intake intake;
    Shooter shooter;
    MyLimeLight limelight;
    Servo blinkin;
    TelemetryManager dashboardTelemetry;
    private boolean isAiming = false;
    private boolean shootReady = false;
    private boolean holdPosition = false;
    private boolean isIntaking = false;
    private boolean intakeForceStop = false;
    private boolean isScanning = false;
    private boolean orderToShoot = false;
    private int aimDirection;
    public static int toggleState = 0;
    public static double xLeftLimit = -6;
    public static double xRightLimit = 4;
    public static double shooterPower = 0.9;
    public SuperSystem(HardwareMap hardwareMap, TelemetryManager dashboard){
        index = new Index(hardwareMap, dashboard);
        ejector = new Ejector(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, dashboard);
        limelight = new MyLimeLight(hardwareMap, dashboard);
        blinkin = hardwareMap.get(Servo.class, "abrahamBlinkin");
        dashboardTelemetry = dashboard;
        reset();
    }
    public void startIntaking(){
        isIntaking = true;
    }

    public void aimShooter(){
        shooterOn();
        isAiming = true;
    }
    public void shoot(){
        orderToShoot = true;
    }

    public void reset(){
        ejector.reset();
        index.reset();
    }

    public void shooterOn(){
        shooter.startShooter();
        shooter.defaultShooterPos();
    }
    public void shooterOff(){
        shooter.stopShooter();
    }

    public void setLED(){
        if(toggleState == 0){
            blinkin.setPosition(1);
        }else if(toggleState == 1){
            blinkin.setPosition(0.5);
        }else if(toggleState == 2){
            blinkin.setPosition(0.722);
        }
    }
    public void toggle(){
        toggleState += 1;
        if (toggleState > 2) {
            toggleState = 0;
        }
        setLED();
    }
    public void forceStopIntake(){
        intakeForceStop = true;
    }
    public boolean intakeIsBusy(){
        return isIntaking;
    }

    public void update(){
        if(isIntaking){
            intake.spinIntake();
            index.setIsSensing(true);
            index.toPickupTarget(0);
            if(index.currentSlotStatus()){
                index.toPickupTarget(0);
            }
            if(index.isFull() || intakeForceStop){
                isIntaking = false;
                intakeForceStop = false;
                index.setIsSensing(false);
//                index.reset();
                intake.stopIntake();
            }
        }
        if(isAiming){
            if (limelight.getxLoc() < xLeftLimit){ //robot moves left
                aimDirection = -1;
            } else if (limelight.getxLoc() > xRightLimit){ //robot moves right
                aimDirection = 1;
            }else{
                aimDirection = 0;
                shootReady = true;
                isAiming = false;
                holdPosition = true;
            }
        }
//        if(shootReady && orderToShoot){
        if(orderToShoot){
            shooter.startShooter();
            if(!index.isEmpty()) {
                if (toggleState == 1 || toggleState == 2) {
                    index.toShootTarget(toggleState);
                    if(index.isAtPosition()){
                        ejector.quickfire();
                        index.emptyCurrentSlot();
                    }
                }else if(toggleState == 0){
                    index.toShootClosest();
                    if(index.isAtPosition()){
                        ejector.quickfire();
                        index.emptyCurrentSlot();
                    }
                }
            }else{
                shooter.stopShooter();
                shootReady = false;
                holdPosition = false;
                orderToShoot = false;
            }
        }
        dashboardTelemetry.addData("isIntaking", isIntaking);
        dashboardTelemetry.addData("forcestop", intakeForceStop);
        dashboardTelemetry.addData("shoot ordered?", orderToShoot);
        setLED();
        index.update();
        ejector.update();
        intake.update();
        shooter.update();
        limelight.update();
    }
    public int getAimDirection(){
        return aimDirection;
    }
    public boolean getAimStatus(){
        return isAiming;
    }
    public boolean holdingPosition(){
        return holdPosition;
    }
    public void startLimelight(int pl){
        limelight.start(pl);
    }
}
