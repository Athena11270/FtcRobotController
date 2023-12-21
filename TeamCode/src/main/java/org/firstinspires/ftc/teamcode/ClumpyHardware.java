package org.firstinspires.ftc.teamcode;

import android.print.PrinterInfo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;

public class ClumpyHardware
{
    //TODO: Grab necessary code from TestTensorFlow1 or TestTensorFlowEasy1
    //TODO: Add camera init stuff to our initialize method
    //TODO: In Autonomous program, write code to drive to first aND SECOND POSITION AND TRY TO DETECT PIXEL
    //todo: WRITE CODE FOR iF IN FIRST POSITION (DRIVE THERE, PLACE) ELSE IF IN SECOND POSITION (DRIVE THERE, PLACE) eLSE ASSUME IT'S IN 3RD POSITION
    private DcMotorEx FL = null;
    private DcMotorEx BL = null;
    private DcMotorEx FR = null;
    private DcMotorEx BR = null;

    // ARM1 needs a rev expansion hub


    private WebcamName WEBCAM1 = null;

    public double TurboBoost = 0.3;

    // The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;

    // The variable to store our instance of the vision portal.
    private VisionPortal visionPortal;





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


    public ClumpyHardware(LinearOpMode opModeReference)
    {
        OpModeReference = opModeReference;
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(WEBCAM1);
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }
    public void Initialize() {
        FL = OpModeReference.hardwareMap.get(DcMotorEx.class, "FL");
        BL = OpModeReference.hardwareMap.get(DcMotorEx.class, "BL");
        FR = OpModeReference.hardwareMap.get(DcMotorEx.class, "FR");
        BR = OpModeReference.hardwareMap.get(DcMotorEx.class, "BR");

        WEBCAM1 = OpModeReference.hardwareMap.get(WebcamName.class, "Webcam 1");

        // USUALLY WE DON"T PROGRAM AROUND HARDWARE PROBLEMS BUT FL IS REVERSED POLARITY
        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.FORWARD);
        BR.setDirection(DcMotorEx.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        initTfod();

    }

    public boolean IsPixel() {
        List<Recognition> currentRecognitions = null;
        boolean found = false;

        for (int x = 0; x < 100; x ++) {
            currentRecognitions = tfod.getRecognitions();
            OpModeReference.sleep(10);
            if (currentRecognitions.size() > 0)
            {
                found = true;
                break;
            }
            OpModeReference.telemetry.addData("loop", x);
        }
        return found;

    }




    public void DriveCM(double power, double cm)
    {
        // calculate number of ticks we want to move
        double targetTicks = TicksPerCM * cm;

        // reset encoders to zero
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // set our target ticks
        FR.setTargetPosition((int)Math.round(targetTicks));
        BR.setTargetPosition((int)Math.round(targetTicks));
        FL.setTargetPosition((int)Math.round(targetTicks));
        BL.setTargetPosition((int)Math.round(targetTicks));

        // set motor mode to run to position
        FR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

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
        FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }

    public void StrafeLeftCM(double power, double cm)
    {
        // calculate number of ticks we want to move
        double targetTicks = TicksPerCM * cm;

        // reset encoders to zero
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // set our target ticks
        FR.setTargetPosition((int)Math.round(targetTicks));
        BR.setTargetPosition((int)Math.round(-targetTicks));
        FL.setTargetPosition((int)Math.round(-targetTicks));
        BL.setTargetPosition((int)Math.round(targetTicks));

        // set motor mode to run to position
        FR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // turn motors on
        FR.setPower(power);
        BR.setPower(-power);
        FL.setPower(-power);
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
        FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void StrafeRightCM(double power, double cm)
    {
        // calculate number of ticks we want to move
        double targetTicks = TicksPerCM * cm;

        // reset encoders to zero
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // set our target ticks
        FR.setTargetPosition((int)Math.round(-targetTicks));
        BR.setTargetPosition((int)Math.round(targetTicks));
        FL.setTargetPosition((int)Math.round(targetTicks));
        BL.setTargetPosition((int)Math.round(-targetTicks));

        // set motor mode to run to position
        FR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

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
        FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }


    public void RunMecanumDrive() {
        double max;

        double normalSpeed = 0.4;
        double maxBoostSpeed = 1 - normalSpeed;

        // speed will increase based on how hard the trigger is pressed
        // determine if trigger has been pressed at all
        triggerPressed = OpModeReference.gamepad1.right_trigger > 0;

        // output of right trigger is (we think) in the range of 0..1
        if (triggerPressed)
        {
            double rightTrigger = OpModeReference.gamepad1.right_trigger;

            TurboBoost = normalSpeed  + rightTrigger * maxBoostSpeed ;
            //OpModeReference.gamepad1.rumble(0.1,0.1,1); //SOME OPTIONAL CHANGES: Varible acelleration with RT, Rumble, Rumble based on RT
            OpModeReference.gamepad1.rumble(1,1,100);
        }else
        {
            TurboBoost = normalSpeed;
        }

        // to swap main stick from left to right (or right to left)
        // press dpad right
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



    public void TimedLeftMoterStop(long Time) {


        try {
            Thread.sleep(Time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        FL.setPower(0);
    }
}
