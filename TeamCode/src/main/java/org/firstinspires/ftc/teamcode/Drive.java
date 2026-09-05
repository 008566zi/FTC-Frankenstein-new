package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive", group = "samples")
public class Drive extends BaseClass { // extends base instead of linearopmode
        @Override
        public void runOpMode() throws InterruptedException {

            initHardware(); // inits hardware
/*            telemetry.addData("Status", "Initialized");
            telemetry.update(); */

            waitForStart();
            matchTime.reset();

            while (opModeIsActive()) {
                double drive;
                double turn;
                double strafe;
                double fLeftPow, fRightPow, bLeftPow, bRightPow;

                // Reverse the right side motors
                // Reverse left motors if you are using NeveRests
                frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
                backRightMotor.setDirection(DcMotor.Direction.REVERSE);

                drive =gamepad1.left_stick_y *-1;
                turn =gamepad1.right_stick_x;
                strafe =gamepad1.left_stick_x;

                fLeftPow =Range.clip(drive +turn +strafe,-1,1);
                bLeftPow =Range.clip(drive +turn -strafe,-1,1);
                fRightPow =Range.clip(drive -turn -strafe,-1,1);
                bRightPow =Range.clip(drive -turn +strafe,-1,1);

                frontLeftMotor.setPower(fLeftPow);
                backLeftMotor.setPower(bLeftPow);
                frontRightMotor.setPower(fRightPow);
                backRightMotor.setPower(bRightPow);
            }
            }
        }
