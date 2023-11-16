package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware
{
    //TODO: Grab necessary code from TestTensorFlow1 or TestTensorFlowEasy1
    //TODO: Add camera init stuff to our initialize method
    //TODO: In Autonomous program, write code to drive to first aND SECOND POSITION AND TRY TO DETECT PIXEL
    //todo: WRITE CODE FOR iF IN FIRST POSITION (DRIVE THERE, PLACE) ELSE IF IN SECOND POSITION (DRIVE THERE, PLACE) eLSE ASSUME IT'S IN 3RD POSITION
    private DcMotorEx FL = null;
    private DcMotorEx BL = null;
    private DcMotorEx FR = null;
    private DcMotorEx BR = null;

    public double TurboBoost = 0.3;




    private LinearOpMode OpModeReference = null;

    private boolean triggerPressed;
    private boolean MainStickLeft = true;

    double DiameterCM = 9.6;
    double CircumferenceCM = DiameterCM * Math.PI;
    double TicksPerRev = 384.5;
    double TicksPerCM = TicksPerRev / CircumferenceCM;

    double axial;
    double lateral;
    double yaw;


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

    public void StrafeLeftCM(double power, double cm) {
        //TODO: write code
    }

    public void StrafeRightCM(double power, double cm) {
        //TODO: write code
    }

    public void DriveCM(double power, double cm)
    {
        // calculate number of ticks we want to move
        double targetTicks = TicksPerCM * cm;

        // reset encoders to zero
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set our target ticks
        FR.setTargetPosition((int)Math.round(targetTicks));
        BR.setTargetPosition((int)Math.round(targetTicks));
        FL.setTargetPosition((int)Math.round(targetTicks));
        BL.setTargetPosition((int)Math.round(targetTicks));

        // set motor mode to run to position
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // turn motors on
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(power);
        BL.setPower(power);

        // wait while motors go to where they need to
        while (FR.isBusy() || BR.isBusy() || FL.isBusy() || BL.isBusy()) {
            // do nothing
        }

        // stop motors
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);

        // set mode back to normal
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void RunMecanumDrive() {
        double max;

        triggerPressed = OpModeReference.gamepad1.right_trigger > 0;

        if (triggerPressed)
        {
            double rightTrigger = OpModeReference.gamepad1.right_trigger;

            TurboBoost = .4  + rightTrigger ;
            //OpModeReference.gamepad1.rumble(0.1,0.1,1); //SOME OPTIONAL CHANGES: Varible acelleration with RT, Rumble, Rumble based on RT
            //OpModeReference.gamepad1.rumble(rightTrigger,rightTrigger,10);
        }else
        {
            TurboBoost = .4;
        }


        if (OpModeReference.gamepad1.dpad_right)
        {
          if (MainStickLeft){
            MainStickLeft = false;
          }
          else if(!MainStickLeft)
          {
              MainStickLeft = true;
          }
        }



        if (MainStickLeft)
        {
             axial   = -OpModeReference.gamepad1.left_stick_y * TurboBoost;  // Note: pushing stick forward gives negative value
             lateral =  OpModeReference.gamepad1.left_stick_x * TurboBoost;//might not work, switch Lefts and Rights to fix
             yaw     =  OpModeReference.gamepad1.right_stick_x * TurboBoost;
        }else
        {
             axial   = -OpModeReference.gamepad1.right_stick_y * TurboBoost;  // Note: pushing stick forward gives negative value
             lateral =  OpModeReference.gamepad1.right_stick_x * TurboBoost;//might not work, switch Lefts and Rights to fix
             yaw     =  OpModeReference.gamepad1.left_stick_x * TurboBoost;
        }



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
