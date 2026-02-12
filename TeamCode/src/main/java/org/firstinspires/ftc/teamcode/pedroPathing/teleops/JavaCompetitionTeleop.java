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
    private double rotSpeed = 0.5;
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

        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("direction facing", drive.getHeadingToMaintain());
        dashboardTelemetry.update();
        if (!superSystem.getAimStatus() && !superSystem.holdingPosition()){
            drive.drive2(digitalTransmission(left_x), digitalTransmission(left_y), -right_x);
        }else if(superSystem.getAimStatus()){
            drive.drive2(digitalTransmission(0), digitalTransmission(0), rotSpeed* superSystem.getAimDirection());
        }else if(superSystem.holdingPosition()){
            drive.drive2(0,0,0);
        }

        if (this.g1.wasJustPressed(GamepadKeys.Button.A)) { //really X
            drive.setHeadingToMaintain(0); // 180 degrees???
        } else if (this.g1.wasJustPressed(GamepadKeys.Button.B)) { //really O
            drive.setHeadingToMaintain(90);
        } else if (this.g1.wasJustPressed(GamepadKeys.Button.Y)) { //really ^
            drive.setHeadingToMaintain(180);
        } else if (this.g1.wasJustPressed(GamepadKeys.Button.X)) { ////really []
            drive.setHeadingToMaintain(-90);
        }

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            superSystem.toggle();
        }

        if (g1.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
            if (this.g1.isDown(GamepadKeys.Button.A)) {
                superSystem.reset();
            }else if(this.g1.isDown(GamepadKeys.Button.B)){

            }else if(this.g1.isDown(GamepadKeys.Button.X)){
                superSystem.startIntaking();
            }else if(this.g1.isDown(GamepadKeys.Button.Y)){
                superSystem.aimShooter();
            }
        }
        superSystem.update();
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double zeroAnalogInput(double input){
        if (Math.abs(input) < 0.05){
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
