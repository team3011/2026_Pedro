package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.SuperSystem;

@Configurable       //if you want configurable constants
@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public abstract class JavaCompetitionTeleop extends OpMode {
    // Declare OpMode members.
    protected AllianceColor allianceColor;
    private ElapsedTime runtime = new ElapsedTime();
    //this section allows us to access telemetry data from a browser
    PanelsTelemetry dashboard = PanelsTelemetry.INSTANCE;
    TelemetryManager dashboardTelemetry = dashboard.getTelemetry();
    double left_y, right_y, left_x, right_x, left_t, right_t;
    MecanumDrive drive;
    GamepadEx g1;
    SuperSystem superSystem;
    public static double rotSpeed = 0.15;
    private boolean snappingToHeading = false;
    private double snapTargetDeg = 0;
    /*     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dashboardTelemetry.addData("Status", "Initialized");
        drive = new MecanumDrive(hardwareMap);
        g1 = new GamepadEx(gamepad1);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step
        superSystem = new SuperSystem(hardwareMap, dashboardTelemetry);
        if(getAllianceColor().equals(AllianceColor.RED)){
            superSystem.setAllianceColor(1);
        }else if(getAllianceColor().equals(AllianceColor.BLUE)){
            superSystem.setAllianceColor(0);
        }
        superSystem.shooterOff();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        superSystem.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //update gamepad values
        g1.readButtons();
        left_y = zeroAnalogInput(g1.getLeftY());
        right_y = zeroAnalogInput(g1.getRightY());
        left_x = zeroAnalogInput(g1.getLeftX());
        right_x = zeroAnalogInput(g1.getRightX());
        left_t = -zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        right_t = zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
//        superSystem.findTargetHeading(drive.getHeadingToMaintain());

        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("direction facing", drive.getHeadingToMaintain());
        dashboardTelemetry.update();
//        if (!superSystem.getAimStatus() && !superSystem.holdingPosition() && !superSystem.getIsIntaking()){
//            drive.drive2(digitalTransmission(-left_x), digitalTransmission(-left_y), right_x);
//        }else if(superSystem.getAimStatus()){
//            drive.drive2(digitalTransmission(0), digitalTransmission(0), rotSpeed * superSystem.getAimDirection());
//        }else if(superSystem.holdingPosition()){
//            drive.drive2(0,0,0);
//        }else if(superSystem.getIsIntaking()){
//            drive.drive2(digitalTransmission(-left_x)/2, digitalTransmission(-left_y)/2, right_x/2);
//        }

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            superSystem.toggle();
        }

        if (superSystem.getAimStatus()) {
            drive.drive2(0, 0, rotSpeed * superSystem.getAimDirection());
        } else if (superSystem.holdingPosition()) {
            drive.drive2(0, 0, 0);
        } else if (superSystem.getIsIntaking()) {
            drive.drive2(digitalTransmission(-left_x)/2, digitalTransmission(-left_y)/2, right_x/2);
        } else if (snappingToHeading && !g1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            double error = snapTargetDeg - drive.getHeadingInDEG();
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            if (Math.abs(error) < 3.0) {
                snappingToHeading = false;
            } else {
                drive.rotateToHeading(snapTargetDeg, digitalTransmission(-left_x), digitalTransmission(-left_y));
            }
        } else {
            drive.drive2(digitalTransmission(-left_x), digitalTransmission(-left_y), right_x);
        }

        if (g1.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            if (this.g1.wasJustPressed(GamepadKeys.Button.A)) {
                superSystem.reset();
            }else if(this.g1.wasJustPressed(GamepadKeys.Button.B)){
                if (getAllianceColor().equals(AllianceColor.RED)) {
                    superSystem.startLimelight(1);
                } else if (getAllianceColor().equals(AllianceColor.BLUE)) {
                    superSystem.startLimelight(0);
                }
                superSystem.shoot();
            }else if(this.g1.wasJustPressed(GamepadKeys.Button.X)){
                if(!superSystem.intakeIsBusy()){
                    superSystem.startIntaking();
                }else{
                    superSystem.forceStopIntake();
                }
            }else if(this.g1.wasJustPressed(GamepadKeys.Button.Y)){
                if(!superSystem.getAimStatus()) {
                    if (getAllianceColor().equals(AllianceColor.RED)) {
                        superSystem.startLimelight(1);
                    } else if (getAllianceColor().equals(AllianceColor.BLUE)) {
                        superSystem.startLimelight(0);
                    }
                    superSystem.aimShooter();
                }else{
                    superSystem.forceStopAiming();
                }
            }else if(this.g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                superSystem.reverseIntake();
            }
        }else{
//            if (this.g1.wasJustPressed(GamepadKeys.Button.A)) { //really X
//                drive.rotateToHeading(0,digitalTransmission(-left_x), digitalTransmission(-left_y)); // 180 degrees???
//            } else if (this.g1.wasJustPressed(GamepadKeys.Button.B)) { //really O
//                drive.rotateToHeading(90,digitalTransmission(-left_x), digitalTransmission(-left_y));
//            } else if (this.g1.wasJustPressed(GamepadKeys.Button.Y)) { //really ^
//                drive.rotateToHeading(180,digitalTransmission(-left_x), digitalTransmission(-left_y));
//            } else if (this.g1.wasJustPressed(GamepadKeys.Button.X)) { ////really []
//                drive.rotateToHeading(-90,digitalTransmission(-left_x), digitalTransmission(-left_y));
//            }
            if (this.g1.wasJustPressed(GamepadKeys.Button.A)) {
                snappingToHeading = true;
                snapTargetDeg = 0;
            } else if (this.g1.wasJustPressed(GamepadKeys.Button.B)) {
                snappingToHeading = true;
                snapTargetDeg = 90;
            } else if (this.g1.wasJustPressed(GamepadKeys.Button.Y)) {
                snappingToHeading = true;
                snapTargetDeg = 180;
            } else if (this.g1.wasJustPressed(GamepadKeys.Button.X)) {
                snappingToHeading = true;
                snapTargetDeg = -90;
            }
            if (Math.abs(right_x) > 0.2) {
                snappingToHeading = false;
            }
        }
        superSystem.update();
        dashboardTelemetry.addData("right bumper", g1.isDown(GamepadKeys.Button.RIGHT_BUMPER));
        double yaw = drive.calcYaw();
        dashboardTelemetry.addData("yaw", yaw);
        dashboardTelemetry.addData("yaw in degrees", drive.getHeadingInDEG());
        dashboardTelemetry.addData("shorter", drive.figureOutWhatIsShorter(Math.toDegrees(yaw)));
        dashboardTelemetry.addData("rotspeed", drive.getRotSpeed());
        dashboardTelemetry.addData("headingToMaintain", drive.getHeadingToMaintain());
        dashboardTelemetry.addData("leftx", left_x);
        dashboardTelemetry.addData("lefty", left_y);
        dashboardTelemetry.addData("filteredDerivative", drive.getFilteredDerivative());
        dashboardTelemetry.addData("kPterm", drive.getKPTerm());
        dashboardTelemetry.addData("kDterm", drive.getKDTerm());
//        dashboardTelemetry.addData("fl Pow", drive.getFlPow());
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double zeroAnalogInput(double input){
        if (Math.abs(input) < 0.2){
            input = 0;
        }
        return input;
    }

    private double digitalTransmission(double input) {
        if (input < -0.8){
            return 3*input+2;
        } else if (input > 0.8){
            return 3*input-2;
        }
        return .5*input;
    }

    protected abstract AllianceColor getAllianceColor();
}
