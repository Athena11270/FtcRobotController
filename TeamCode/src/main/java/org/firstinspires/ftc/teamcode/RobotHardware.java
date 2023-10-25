package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
public class RobotHardware
{
    private DcMotorEx FL = null;
    private DcMotorEx BL = null;
    private DcMotorEx FR = null;
    private DcMotorEx BR = null;

    private LinearOpMode OpModeReference = null;

    public RobotHardware(LinearOpMode opModeReference)
    {
        OpModeReference = opModeReference;
    }

    public void Initialize() {
        FL = OpModeReference.hardwareMap.get(DcMotorEx.class, "FL");
        BL = OpModeReference.hardwareMap.get(DcMotorEx.class, "BL");
        FR = OpModeReference.hardwareMap.get(DcMotorEx.class, "FR");
        BR = OpModeReference.hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
    }

    public void MecanumDrive() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -OpModeReference.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  OpModeReference.gamepad1.left_stick_x;
        double yaw     =  OpModeReference.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }


        // Send calculated power to wheels
        FL.setPower(leftFrontPower);
        FR.setPower(rightFrontPower);
        BL.setPower(leftBackPower);
        BR.setPower(rightBackPower);
    }


}
