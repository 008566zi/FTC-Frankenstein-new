package org.firstinspires.ftc.teamcode.C_Bot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RunningAverageArray;

/**
 * Iterative Tele OpMode for a two wheel robot
 */
@TeleOp(name="C TELEOP")
//@Disabled
public class C_Tele extends OpMode
{
    // Declare OpMode members.
    private C_TWB twb;

    private RunningAverageArray joystickS; // to smooth aggressive joystick inputs


    /**
     * run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        twb = new C_TWB(hardwareMap); // Create twb object

        joystickS = new RunningAverageArray(12,false); // initialize size of running average

        twb.init();
    }

    /**
     * run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addLine("INIT LOOP");
        twb.init_loop();
        twb.writeTelemetry(this);
        telemetry.update();
        twb.setFlywheel(0.0);
    }

    /**
     * run ONCE when the driver hits START
     */
    @Override
    public void start() {
        twb.start();
    }

    /**
     * run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.rightBumperWasPressed()) twb.collectFlywheel();
        //if (gamepad1.y) twb.shootFlywheel();
        if (gamepad1.leftBumperWasReleased()) twb.flywheelOff();

        // allow for variation of the max velocity
        twb.setMaxSpeedGamepad(this);

        // Use running average of the joystick to smooth aggressive inputs.
        joystickS.add(gamepad1.left_stick_y);

        // The left trigger is a speed booster
        //joystickS.add(gamepad1.left_stick_y * (1 + gamepad1.left_trigger/2.0));

        // Translate the robot by setting position, velocity and pitch targets
        twb.translateDrive(joystickS.getAverage(), twb.getDEGPLoop());

        // Either joystick can turn the robot.  Different speeds. Sets yaw target
        twb.turn_teleop(-gamepad1.left_stick_x * 0.03);
        twb.turn_teleop(-gamepad1.right_stick_x * 0.04);

        twb.loopC(this);  // call the MAIN CONTROL SYSTEM

        if(gamepad1.backWasPressed()) { // toggle gear state
            if (twb.isGearDown()) twb.moveGearUp();
            else  twb.moveGearDown();
        }

        //twb.writeTelemetry(this);
        telemetry.update();
    }
}