package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Drive Module: Handles the math and logic for Mecanum movement.
 * This is now a component, not an OpMode.
 */
public class Drive {
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    public double speedMultiplier = 1.0;
    private boolean lastRB = false;
    private boolean lastLB = false;

    public Drive(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        this.frontLeftMotor = frontLeft;
        this.backLeftMotor = backLeft;
        this.frontRightMotor = frontRight;
        this.backRightMotor = backRight;

        // Initial setup for directions
        this.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        this.backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Updates motor powers based on gamepad input.
     * Should be called repeatedly in the OpMode loop.
     */
    public void update(Gamepad gamepad) {
        // Adjust speed level with bumpers (debounced edge-detection)
        boolean currentRB = gamepad.right_bumper;
        boolean currentLB = gamepad.left_bumper;

        if (currentRB && !lastRB) {
            speedMultiplier = Math.min(1.0, speedMultiplier + 0.1);
        }
        if (currentLB && !lastLB) {
            speedMultiplier = Math.max(0.1, speedMultiplier - 0.1);
        }

        lastRB = currentRB;
        lastLB = currentLB;

        double drive = gamepad.left_stick_y * -1;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;

        double fLeftPow = Range.clip((drive + turn + strafe) * speedMultiplier, -1, 1);
        double bLeftPow = Range.clip((drive + turn - strafe) * speedMultiplier, -1, 1);
        double fRightPow = Range.clip((drive - turn - strafe) * speedMultiplier, -1, 1);
        double bRightPow = Range.clip((drive - turn + strafe) * speedMultiplier, -1, 1);

        frontLeftMotor.setPower(fLeftPow);
        backLeftMotor.setPower(bLeftPow);
        frontRightMotor.setPower(fRightPow);
        backRightMotor.setPower(bRightPow);
    }
}
