package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Linear Slide Module: Handles safe controls for dual-motor linear slides.
 * Uses encoder targets to ensure synchronous lift.
 */
public class Linear {
    private final DcMotor leftSlide;
    private final DcMotor rightSlide;

    // Safety limits for standard goBILDA Linear Actuator Kit (1120 Series)
    // - Motor: 5.2:1 Yellow Jacket (145.1 Ticks per Output Revolution)
    // - Lead Screw: 8mm travel per output revolution
    // - Resolution: ~18.14 ticks per mm
    // - Stroke Length: 203mm max travel -> 203mm * 18.14 ticks/mm = ~3682 ticks
    private static final int MIN_POSITION = 0;
    private static final int MAX_POSITION = 3680; 
    private static final double SLIDE_POWER = 1.0; // Linear actuators require full power for heavy thrust loads

    private int targetPosition = 0;
    private boolean lastTriggerUp = false;
    private boolean lastTriggerDown = false;

    public Linear(DcMotor leftSlide, DcMotor rightSlide) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;

        // Reset and prepare encoders
        this.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse one side if motors are mounted mirrored facing each other
        this.rightSlide.setDirection(DcMotor.Direction.REVERSE);

        this.leftSlide.setTargetPosition(MIN_POSITION);
        this.rightSlide.setTargetPosition(MIN_POSITION);

        this.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Updates slide positions using manual joystick overrides or button presets.
     * Call this inside your main OpMode loop.
     */
    public void update(Gamepad gamepad) {
        // Option 1: Manual control using left stick Y (with safety limits)
        double manualInput = -gamepad.left_stick_y; // Invert stick direction
        
        if (Math.abs(manualInput) > 0.05) {
            // Scale step increment based on joystick tilt
            targetPosition += (int) (manualInput * 40); 
        } 
        // Option 2: Preset increments with Triggers (Analog threshold > 0.5 counts as pressed)
        else {
            boolean currentTriggerUp = gamepad.left_trigger > 0.5;
            boolean currentTriggerDown = gamepad.right_trigger > 0.5;

            if (currentTriggerUp && !lastTriggerUp) {
                targetPosition = MAX_POSITION; // Fully extend
            } else if (currentTriggerDown && !lastTriggerDown) {
                targetPosition = MIN_POSITION; // Fully retract
            }

            lastTriggerUp = currentTriggerUp;
            lastTriggerDown = currentTriggerDown;
        }

        // Apply strict safety bounds to prevent breaking string/chains
        targetPosition = Range.clip(targetPosition, MIN_POSITION, MAX_POSITION);

        // Feed target positions to the motor controllers
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);

        // Maintain constant holding power
        leftSlide.setPower(SLIDE_POWER);
        rightSlide.setPower(SLIDE_POWER);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getLeftCurrentPosition() {
        return leftSlide.getCurrentPosition();
    }

    public int getRightCurrentPosition() {
        return rightSlide.getCurrentPosition();
    }

    /**
     * Emergency / Shutdown retraction: Pulls the slide down to zero position safely on exit.
     * Uses a brief block window since the FTC SDK terminates hardware links quickly.
     */
    public void retractOnStop() {
        targetPosition = MIN_POSITION;
        leftSlide.setTargetPosition(MIN_POSITION);
        rightSlide.setTargetPosition(MIN_POSITION);
        
        leftSlide.setPower(0.6); // Use a gentle speed for auto-retract
        rightSlide.setPower(0.6);

        // Keep driving down briefly while it's moving towards 0
        long startTime = System.currentTimeMillis();
        while ((leftSlide.isBusy() || rightSlide.isBusy()) && (System.currentTimeMillis() - startTime < 1500)) {
            // Wait up to 1.5 seconds maximum or until encoders reach 0
            if (Math.abs(leftSlide.getCurrentPosition()) < 15 && Math.abs(rightSlide.getCurrentPosition()) < 15) {
                break;
            }
        }
        
        // Turn off motors completely
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }
}
