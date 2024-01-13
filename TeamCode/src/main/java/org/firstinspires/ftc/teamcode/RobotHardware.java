package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

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

    private Servo ReleaseServo = null;
    private Servo CLAW = null;


    // ARM1 needs a rev expansion hub
    private DcMotorEx ARM = null;
    private DcMotorEx INTAKESPIN = null;
    private DcMotorEx INTAKELIFT = null;

    private WebcamName WEBCAM1 = null;

    public double TurboBoost = 0.3;

    // The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;

    // The variable to store our instance of the vision portal.
    private VisionPortal visionPortal;





    private LinearOpMode OpModeReference = null;

    private boolean triggerPressed;
    private boolean MainStickLeft = true;
    private boolean clawClosed = false;

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
        ARM = OpModeReference.hardwareMap.get(DcMotorEx.class, "ARM");
        INTAKELIFT = OpModeReference.hardwareMap.get(DcMotorEx.class, "INTAKELIFT");
        INTAKESPIN = OpModeReference.hardwareMap.get(DcMotorEx.class, "INTAKESPIN");
        WEBCAM1 = OpModeReference.hardwareMap.get(WebcamName.class, "Webcam 1");
        ReleaseServo = OpModeReference.hardwareMap.get(Servo.class, "DUMPER");
        CLAW = OpModeReference.hardwareMap.get(Servo.class, "CLAW");


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

            TurboBoost = normalSpeed  + (rightTrigger * maxBoostSpeed) ;
            //OpModeReference.gamepad1.rumble(0.1,0.1,1); //SOME OPTIONAL CHANGES: Varible acelleration with RT, Rumble, Rumble based on RT
            OpModeReference.gamepad1.rumble(.1,.1,100);
        }else
        {
            TurboBoost = normalSpeed;
        }

        // to swap main stick from left to right (or right to left)
        // press dpad right
//        if (OpModeReference.gamepad1.dpad_right)
//        {
//          if (MainStickLeft){
//            MainStickLeft = false;
//          }
//          else if(!MainStickLeft)
//          {
//              MainStickLeft = true;
//          }
//        }


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

    public void ArmControl() {

        // press cross to make arm go up
        // press circle to make arm go down
        if (OpModeReference.gamepad2.cross) {
            ARM.setPower(0.4);
        }
        else if (OpModeReference.gamepad2.circle) {
            ARM.setPower(-0.4);
        }
        else {
            ARM.setPower(0);
        }
    }

    public void DumpPixel() {
        ReleaseServo.setPosition(1);
        OpModeReference.sleep(1000);
        ReleaseServo.setPosition(0);
    }

    public void Dumper() {
        // pixel release servo
        if (OpModeReference.gamepad1.square) {
            DumpPixel();
        }
    }

    public void Grabber() {
        if (OpModeReference.gamepad2.dpad_down)
        {
            CLAW.setPosition(0.5);
        } else //if (!OpModeReference.gamepad2.dpad_down)
        {
            CLAW.setPosition(0);
        }
    }
    // *** What does this do? ***
    // *** Looks like it stops ONE of the wheels after a pause. WHY??? ***
   /* public void TimedLeftMoterStop(long Time) {
        try {
            Thread.sleep(Time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        FL.setPower(0);
    }

    */
    public void IntakeLiftControl() {
        if (OpModeReference.gamepad2.triangle) {
            INTAKELIFT.setPower(.3);
        }
        else if (OpModeReference.gamepad2.square) {
            INTAKELIFT.setPower(-.3 );
        }
        else {
            INTAKELIFT.setPower(0);
        }
    }

//    public void TimedIntakeLiftControl(long Time) {
//        INTAKELIFT.setPower(.6);
//
//      try {
//          Thread.sleep(Time);
//            }
//              catch (InterruptedException e)
//            {
//                e.printStackTrace();
//            }
//
//        INTAKELIFT.setPower(0);
//
//    }


    public void IntakeSpinControl() {
        if (OpModeReference.gamepad2.left_bumper) {
            INTAKESPIN.setPower(0.5);
        }
        else if (OpModeReference.gamepad2.right_bumper) {
            INTAKESPIN.setPower(-.8);
        }
        else {
            INTAKESPIN.setPower(0);
        }
    }



//    public class TestTensorFlow1 extends LinearOpMode {
//
//        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//        // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
//        // this is only used for Android Studio when using models in Assets.
//        private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
//        // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
//        // this is used when uploading models directly to the RC using the model upload interface.
//        private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
//        // Define the labels recognized in the model for TFOD (must be in training order!)
//        private static final String[] LABELS = {
//                "Pixel",
//        };
//
//        /**
//         * The variable to store our instance of the TensorFlow Object Detection processor.
//         */
//        private TfodProcessor tfod;
//
//        /**
//         * The variable to store our instance of the vision portal.
//         */
//        private VisionPortal visionPortal;
//
//        @Override
//        public void runOpMode() {
//
//            initTfod();
//
//            // Wait for the DS start button to be touched.
//            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//            telemetry.addData(">", "Touch Play to start OpMode");
//            telemetry.update();
//            waitForStart();
//
//            if (opModeIsActive()) {
//                while (opModeIsActive()) {
//
//                    telemetryTfod();
//
//                    // Push telemetry to the Driver Station.
//                    telemetry.update();
//
//                    // Save CPU resources; can resume streaming when needed.
//                    if (gamepad1.dpad_down) {
//                        visionPortal.stopStreaming();
//                    } else if (gamepad1.dpad_up) {
//                        visionPortal.resumeStreaming();
//                    }
//
//                    // Share the CPU.
//                    sleep(20);
//                }
//            }
//
//            // Save more CPU resources when camera is no longer needed.
//            visionPortal.close();
//
//        }   // end runOpMode()
//
//        /**
//         * Initialize the TensorFlow Object Detection processor.
//         */
//        private void initTfod() {
//
//            // Create the TensorFlow processor by using a builder.
//            tfod = new TfodProcessor.Builder()
//
//                    // With the following lines commented out, the default TfodProcessor Builder
//                    // will load the default model for the season. To define a custom model to load,
//                    // choose one of the following:
//                    //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                    //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                    //.setModelAssetName(TFOD_MODEL_ASSET)
//                    //.setModelFileName(TFOD_MODEL_FILE)
//
//                    // The following default settings are available to un-comment and edit as needed to
//                    // set parameters for custom models.
//                    //.setModelLabels(LABELS)
//                    //.setIsModelTensorFlow2(true)
//                    //.setIsModelQuantized(true)
//                    //.setModelInputSize(300)
//                    //.setModelAspectRatio(16.0 / 9.0)
//
//                    .build();
//
//            // Create the vision portal by using a builder.
//            VisionPortal.Builder builder = new VisionPortal.Builder();
//
//            // Set the camera (webcam vs. built-in RC phone camera).
//            if (USE_WEBCAM) {
//                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//            } else {
//                builder.setCamera(BuiltinCameraDirection.BACK);
//            }
//
//            // Choose a camera resolution. Not all cameras support all resolutions.
//            //builder.setCameraResolution(new Size(640, 480));
//
//            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//            builder.enableLiveView(true);
//
//            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//            // Choose whether or not LiveView stops if no processors are enabled.
//            // If set "true", monitor shows solid orange screen if no processors enabled.
//            // If set "false", monitor shows camera view without annotations.
//            builder.setAutoStopLiveView(false);
//
//            // Set and enable the processor.
//            builder.addProcessor(tfod);
//
//            // Build the Vision Portal, using the above settings.
//            visionPortal = builder.build();
//
//            // Set confidence threshold for TFOD recognitions, at any time.
//            //tfod.setMinResultConfidence(0.75f);
//
//            // Disable or re-enable the TFOD processor at any time.
//            //visionPortal.setProcessorEnabled(tfod, true);
//
//        }   // end method initTfod()
//
//        /**
//         * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
//         */
//        private void telemetryTfod() {
//
//            List<Recognition> currentRecognitions = tfod.getRecognitions();
//            telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//            // Step through the list of recognitions and display info for each one.
//            for (Recognition recognition : currentRecognitions) {
//                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//                telemetry.addData(""," ");
//                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                telemetry.addData("- Position", "%.0f / %.0f", x, y);
//                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//            }   // end for() loop
//
//        }   // end method telemetryTfod()

//    }   // end class

}
