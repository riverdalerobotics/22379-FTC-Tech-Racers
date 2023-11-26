package org.firstinspires.ftc.teamcode.TERbot;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math.*;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name="Aidan Auto", group="Linear OpMode")
public class Auto extends LinearOpMode {



    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor armMotor = null;
    DcMotor clawPivotMotor = null;

    Servo leftClawServo = null;
    Servo rightClawServo = null;


    IMU imu = null;

    private int ticksToDist;


    private int maxClawPivotPosition;
    private int minClawPivotPosition;

    private double maxClawServoPosition;
    private double minClawServoPosition;
    private double clawServosDefaultPosition;



    String driveOrTurn = null;

    private double headingError;
    private double turnSpeed;
    private double driveSpeed;
    private double leftSpeed;
    private double rightSpeed;
    private double targetHeading;


    // 1, 2, or 3
    //1 is left, 2 is center, 3 is right
    private int propLocation;
    private boolean tempPropFound;


    //CONSTANTS//


    static final double P_DRIVE_GAIN = 0.3;
    static final double P_TURN_GAIN = 0.03;
    static final double HEADING_THRESHOLD = 1.0;

    //Wheel circumference is in centimeters
    //Approx 0.314 meters
    static final double WHEEL_CIRCUMFERENCE_METERS = Constants.OdometryConstants.WHEEL_CIRCUMFERENCE_METERS;

    static final int ENCODER_TICKS_TO_REVOLUTION = Constants.OdometryConstants.ENCODER_TICKS_TO_REVOLUTION;


    static final String TFOD_MODEL_ASSET = "TFOD.tflite";



    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);




    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.

    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.

    private static final String[] LABELS = {
            "BlueProp", "RedProp"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private ChassisSubsystem chassis;
    private ArmSubsystem arm;

    @Override
    public void runOpMode() throws InterruptedException {
        propLocation = 0;
        imu = hardwareMap.get(IMU.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        clawPivotMotor = hardwareMap.get(DcMotor.class, "clawPivotMotor");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");



        chassis = new ChassisSubsystem(leftMotor, rightMotor);
        arm = new ArmSubsystem(armMotor, leftClawServo, rightClawServo, clawPivotMotor);
        initTfod();



        RevHubOrientationOnRobot orientationOnRobot = Constants.OdometryConstants.REV_HUB_ORIENTATION;
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();
        imu.resetYaw();



        arm.openClaw();
        arm.closeClaw();
        arm.setArmPivotPosition(150, 0.5);
        //locateTeamPropAndDropPixel();



        while(opModeIsActive()) {
            sendTelemetry();
            telemetry.update();
        }



    }



    private void testAuto2() {
        driveForwardDistance(0.3, 0.3);
        turnToHeading(0.2, 90);
        driveForwardDistance(0.3, 0.3);
    }
    private void testAuto() {
        driveForwardDistance(0.5, 1);
        while(opModeIsActive()) {
            telemetry.addData("BEFOREINIT", true);
            telemetry.update();
            telemetryTfod();
            telemetry.addData("TFODWORKING", runtime.time());
            telemetry.update();
        }
    }
    private void locateTeamPropAndDropPixel() {
        //Drives to the middle point between all 3 spikes. Aligned with the center spike, and equidistant to
        //The 2 side spikes
        goForwardStart();


        double curRunTime = runtime.time();
        //Looks for the prop on the center spike for 2 seconds
        while(runtime.time() < curRunTime + 2) {
            telemetry.addData("TFOD WORKING", "WAHOOOO");
            telemetryTfod();

        }
        //If the prop was located on center spike, move towards it then drop pixels
        if(tempPropFound) {
            propLocation = 1;
            goToCenterSpike();
            //openClaw();
            return;
        }
        //Rotates 30 degrees left
        turnToHeading(0.5, 30);
        curRunTime = runtime.time();
        //Looks for the prop on the left spike for 2 seconds
        while (runtime.time() < curRunTime + 2) {
            telemetryTfod();
            telemetry.addData("left", true);
            telemetry.update();
        }

        //If the prop was located on the left spike, move towards it then drop pixels
        if(tempPropFound && propLocation == 0) {
            propLocation = 2;
            goToSideSpikes();
            //openClaw();
            return;
        }
        //Turns 60 degrees right (30 to -30)
        turnToHeading(0.5, -60);
        curRunTime = runtime.time();
        //Looks for the prop on the right spike for 2 seconds
        while(runtime.time() < curRunTime + 2) {
            telemetryTfod();
        }
        //If the prop was located on the right spike, move towards it then drop pixels
        if(tempPropFound && propLocation == 0) {
            propLocation = 3;
            goToSideSpikes();
            //openClaw();
            return;
        }
    }

    private void goForwardStart() {
        //Approx 24 inches
        driveForwardDistance(0.3, 0.2);
    }

    private void goToSideSpikes() {
        //Approx 3 inches
        //driveForwardDistance(0.5, 0.076);
        driveForwardDistance(0.5, 0.2);
    }
    private void goToCenterSpike() {
        //Approx 6 inches
        driveForwardDistance(0.5, 0.4);
    }





    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)


                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.50f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()



    private void telemetryTfod() {
        tempPropFound = false;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            //If a prop is located consistently while this is being run
            //tempPropFound will be true
            //If it is true at the end of the check, then the prop must be at the check spot
            tempPropFound = true;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()

    private double encoderToMeters(int encVal, int ticksToRotation, double wheelCircumferenceMeters) {
        double numRevolutions = encVal / ticksToRotation;
        double metersTravelled = numRevolutions * wheelCircumferenceMeters;
        return metersTravelled;
    }


    //This method is run when the robot isn't moving
    private int metersToEncoderTicks(double distMeters, int ticksToRotation, double wheelCircumferenceMeters) {
        double numRevolutions = distMeters/wheelCircumferenceMeters;
        int encoderTicks = (int)(numRevolutions * ticksToRotation);
        return encoderTicks;
    }




    private void driveForwardDistance(double power, double distanceMeters) {
        driveOrTurn = "DRIVE";
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ticksToDist = metersToEncoderTicks(distanceMeters, 1440, WHEEL_CIRCUMFERENCE_METERS);
        leftMotor.setTargetPosition(ticksToDist);
        rightMotor.setTargetPosition(ticksToDist);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.moveRobot(power, 0);


        //Signals the desired distance has been achieved
        //Will stay in this loop until the encoders have reached the desired position
        //OLd loop condition: while (leftMotor.isBusy() && rightMotor.isBusy())
        //Wouldnt work consistently
        while (leftMotor.getCurrentPosition() < ticksToDist && rightMotor.getCurrentPosition() < ticksToDist) {
            sendTelemetry();
        }
        chassis.moveRobot(0, 0);
    }


    public void sendTelemetry() {
        telemetry.addData("dist", encoderToMeters(leftMotor.getCurrentPosition(), ENCODER_TICKS_TO_REVOLUTION, WHEEL_CIRCUMFERENCE_METERS));
        telemetry.addData("Heading", getHeading());
        telemetry.addData("Claw piv enc", clawPivotMotor.getCurrentPosition());
        telemetry.addData("Motor piv enc", armMotor.getCurrentPosition());
        telemetry.addData("curRuntime", runtime.time());
        telemetry.addData("Prop location", propLocation);
        telemetry.addData("PropFound?", tempPropFound);
        telemetry.addData("right servo", rightClawServo.getPosition());
        telemetry.addData("left servo", leftClawServo.getPosition());
        telemetry.update();
    }


    public void turnToHeading(double maxTurnSpeed, double heading) {
        driveOrTurn = "TURN";
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            sendTelemetry();



            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            chassis.moveRobot(0, turnSpeed);


        }
        // Stop all motion;
        chassis.moveRobot(0, 0);

        //These lines were active while debugging auto movement
        //Try uncommenting if errors pop up
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry


        // Determine the heading current error
        headingError = targetHeading - getHeading();


        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;


        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}