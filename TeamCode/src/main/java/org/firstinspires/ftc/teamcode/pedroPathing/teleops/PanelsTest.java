package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Config       //if you want config
//@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public class PanelsTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //this section allows us to access telemetry data from a browser
    PanelsTelemetry dashboard = PanelsTelemetry.INSTANCE;
    Telemetry dashboardTelemetry = dashboard.INSTANCE.getFtcTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dashboardTelemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step

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
        dashboardTelemetry.addData("Started", true);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
