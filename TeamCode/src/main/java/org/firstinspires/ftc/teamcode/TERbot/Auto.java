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
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name="Aidan Auto", group="Linear OpMode")
public class Auto extends LinearOpMode {


    private DcMotor frontLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor backLeftDriveMotor;
    private DcMotor backRightDriveMotor;
    DcMotor armMotor;
    DcMotor clawPivotMotor;

    Servo leftClawServo;
    Servo rightClawServo;


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


    double rangeError = 1000;
    double tagHeadingError = 1000;
    double yawError = 1000;
    //CONSTANTS//


    //SPIKE MOVEMENT CONSTANTS
    static final double P_TURN_GAIN = 0.05;
    static final double HEADING_THRESHOLD = 1.0;


    //BOARD APRIL TAG MOVEMENT CONSTANTS
    //DESIRED_DISTANCE is in inches
    static final double DESIRED_INIT_DISTANCE = 16;
    static final double DESIRED_DISTANCE = 10;
    static final double DESIRED_STRAFE_OFFSET = -15;
    static final double P_SPEED_GAIN = 0.1;
    static final double P_STRAFE_GAIN = 0.06;
    static final double P_TAG_TURN_GAIN = 0.06;

    static final double RANGE_THRESHOLD = 0.3;
    static final double TAG_HEADING_THRESHOLD = 1.2;
    static final double STRAFE_THRESHOLD = 0.8;

    static final double MAX_AUTO_SPEED = 0.4;
    static final double MAX_AUTO_TURN = 0.3;
    static final double MAX_AUTO_STRAFE = 0.3;


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

    private int tagToDetect = 3;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

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
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        clawPivotMotor = hardwareMap.get(DcMotor.class, "clawPivotMotor");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");


        chassis = new ChassisSubsystem(frontLeftDriveMotor, frontRightDriveMotor, backLeftDriveMotor, backRightDriveMotor);
        arm = new ArmSubsystem(armMotor, leftClawServo, rightClawServo, clawPivotMotor);
        initTfodAndAprilTag();

        //Manual exposure was causing issues in varying light levels so we opted for automatic exposure
        //which worked much better
        //setManualExposure(4, 250);  // Use low exposure time to reduce motion blur
        setAutoExposure(250);

        RevHubOrientationOnRobot orientationOnRobot = Constants.OdometryConstants.REV_HUB_ORIENTATION;
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        arm.closeClaw();
        waitForStart();
        imu.resetYaw();


        //blueSide45PointAuto();
        redSide45PointAuto();
        //redSide20PointAuto();
        //blueSide20PointAuto();


    }



    private void pushIntoBoardAndReset() {
        //This is the last method called in every auto routine. Once the claw is lined up with the tag
        //the robot raises the arm and moves forward, pushing the claw into the board
        //Once the claw has been opened, the bot moves back slightly, letting the pixel fall and parking.
        //The claw is also reset and prepped for the teleop phase

        //Raise arm and push into board with claw
        arm.setArmPivotPosition(150, 0.5);
        arm.setClawPivotPosition(-100, 0.5);
        driveForwardDistance(0.5, 0.23);

        //Release claw and move back to drop piece
        arm.openRightClaw();
        double curTime = runtime.time();
        while (runtime.time() < curTime + 1) {
        }
        driveForwardDistance(0.3, -0.05);
        arm.closeClaw();
        arm.setArmPivotPosition(Constants.ArmConstants.MIN_ARM_POSITION, 0.5);
        arm.setClawPivotPosition(Constants.ClawConstants.MAX_CLAW_PIVOT_POSITION, 0.5);
    }

    private void redSide45PointAuto() {
        //This auto is used board-side on red. It places the spike and board pixels for 20 points each,
        //and ends parked for 5


        //Raise arm to prevent dragging
        startArmForAuto();


        //Scan for prop and drop pixel on correct spike
        locateTeamPropAndDropPixelRedSide();

        //Once pixel has been dropped on spike, move towards the board
        //RED
        arm.setArmPivotPosition(150, 0.5);
        waitTime(0.5);
        turnToHeading(0.5, -60);
        driveForwardDistance(0.5, 0.45);
        //RED

        //Orient with board and position in front of correct april tag
        locateTagAndScorePixel("red");

        pushIntoBoardAndReset();
        while (opModeIsActive()) {
            sendTelemetry();
            telemetry.update();
        }
    }

    private void redSide20PointAuto() {
        //Used on red side away from the board. This auto simply places the spike pixel for 20, then
        //returns to its starting position

        startArmForAuto();


        //Scan for prop and drop pixel on correct spike
        locateTeamPropAndDropPixelRedSideFar();

        //Once pixel is placed, return to starting position
        turnToHeading(0.5, 0);
        goForwardStart(-1);

        //Loop until auto phase is complete
        while (opModeIsActive()) {
            sendTelemetry();
            telemetry.update();
        }
    }
    private void blueSide20PointAuto() {
        startArmForAuto();


        //Scan for prop and drop pixel on correct spike
        locateTeamPropAndDropPixelBlueSide();
        while (opModeIsActive()) {
            sendTelemetry();
            telemetry.update();
        }
    }
    private void blueSide45PointAuto() {
        //Used on blue board side. Scores 45 points using spike & board pixels + park
        startArmForAuto();


        locateTeamPropAndDropPixelBlueSide();

        //BLUE
        turnToHeading(0.5, 70);
        driveForwardDistance(0.5, 0.45);
        //BLUE

        //Orient robot and position with tag
        locateTagAndScorePixel("blue");

        pushIntoBoardAndReset();
        while (opModeIsActive()) {


            sendTelemetry();
            telemetry.update();
        }


    }

    private void locateTagAndScorePixel(String team) {
        //This method is used by both 45 point autos to line up with the board, then move to the correct
        //april tag and score their yellow pixel

        //This is the distance to line up with the middle april tag for calibration
        double desDist = DESIRED_INIT_DISTANCE;

        //On blue, the bot will line up with 2 first. On red the bot will line up with 5 first (both are middle)
        int tag = 0;
        if (team == "blue") {
            tag = 2;
        }
        else if (team == "red") {
            tag = 5;
        }
        double desHeading = 0;
        double desStrafe = 0;
        boolean moving = true;
        boolean score = false;
        boolean orient = false;
        double timer = 1000000;
        boolean done = false;
        int whichFound = 0;
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!done) {
            telemetry.addData("Heading", getHeading());
            telemetry.addData("tag find", tag);
            telemetry.addData("DIST GO TO", desDist);
            telemetry.addData("HeadingError", tagHeadingError);
            telemetry.addData("rangeError", rangeError);
            telemetry.addData("yaw Error", yawError);
            telemetry.addData("Robot moving", moving);
            telemetry.addData("which found", whichFound);
            telemetry.update();


            if (runtime.time() > timer + 1) {
                score = true;
            }

            if (((runtime.time() < timer + 1 && runtime.time() > timer) || Math.abs(tagHeadingError) > TAG_HEADING_THRESHOLD) || (Math.abs(rangeError) > RANGE_THRESHOLD) || (Math.abs(yawError) > STRAFE_THRESHOLD)) {
                moveToAprilTag(desDist, tag, desStrafe, desHeading);
                moving = true;
            } else {
                chassis.moveRobotMecanum(0, 0, 0);
                moving = false;
                if (score == false) {
                    //imu.resetYaw();
                } else if (score) {
                    if (team == "red") {
                        if (tag == 4 || tag == 5) {
                            turnToHeading(0.5, -88);
                        }
                        else {
                            turnToHeading(0.5, -90);
                        }
                    }
                    else if (team == "blue") {
                        turnToHeading(0.5, 90);
                    }
                    done = true;

                }

            }
            if (moving == false && orient == false) {

                tag = tagToDetect;

                if (tag == 1) {
                    turnToHeading(0.5, 105);
                    whichFound = 1;
                    //turnToHeading(0.5, 15);
                } else if (tag == 3) {
                    //turnToHeading(0.5, 75f);
                    turnToHeading(0.5, 75);
                    whichFound = 3;
                }
                else if (tag == 6) {
                    turnToHeading(0.5, -105);
                    whichFound = 6;
                }
                else if (tag == 4) {
                    turnToHeading(0.5, -75);
                    whichFound = 4;
                }

                desDist = DESIRED_DISTANCE;
                desStrafe = DESIRED_STRAFE_OFFSET;
                desHeading = 0;
                timer = runtime.time();
                moving = true;
                orient = true;
            }
        }
    }


    private void startArmForAuto() {
        //Called at the beginning of all auto rountines to pivot the arm and claw into position

        arm.setArmPivotPosition(120, 0.5);
        //arm.setClawPivotPosition(40, 0.1);


        waitTime(0.5);
        arm.setClawPivotPosition(-270, 0.1);
        waitTime(0.5);
    }



    private void waitTime(double time) {
        double curRunTime = runtime.time();
        while (runtime.time() < curRunTime + time) {

        }
    }

    private void locateTeamPropAndDropPixelBlueSide() {
        //Drives to the middle point between all 3 spikes. Aligned with the center spike, and equidistant to
        //The 2 side spikes. Turns to the center and left to look for the team prop. Will go to whichever spike
        //it finds the prop on and heads to it to drop the spike. After dropping, it returns to the intial scanning
        //position in preparation for moving towards the board
        goForwardStart(1);
        turnToHeading(0.5, 10);


        double curRunTime = runtime.time();
        //Looks for the prop on the center spike for 2 seconds
        while (runtime.time() < curRunTime + 2) {
            telemetry.addData("TFOD WORKING", "WAHOOOO");
            telemetryTfod("BlueProp");

        }
        //If the prop was located on center spike, move towards it then drop pixels
        if (tempPropFound) {
            turnToHeading(0.5, -5);
            propLocation = 2;
            tagToDetect = 2;
            goToCenterSpikeBlue(1);
            waitTime(0.5);
            arm.openLeftClaw();
            waitTime(0.5);
            goToCenterSpikeBlue(-1);
            return;
        }
        //Rotates 30 degrees left
        turnToHeading(0.5, 30);
        curRunTime = runtime.time();
        //Looks for the prop on the left spike for 2 seconds
        while (runtime.time() < curRunTime + 2) {
            telemetryTfod("BlueProp");
            telemetry.addData("left", true);
            telemetry.update();
        }

        //If the prop was located on the left spike, move towards it then drop pixels
        if (tempPropFound && propLocation == 0) {
            propLocation = 1;
            tagToDetect = 1;
            turnToHeading(0.5, 15);
            goToLeftSpikeBlue(1);
            waitTime(0.5);
            arm.openLeftClaw();
            waitTime(0.5);
            goToLeftSpikeBlue(-1);
            return;
        }
        propLocation = 3;
        tagToDetect = 3;
        turnToHeading(0.5, -30);
        goToRightSpikeBlue(1);
        turnToHeading(0.5, -45);
        waitTime(0.5);
        arm.openLeftClaw();
        waitTime(0.5);
        turnToHeading(0.5, -30);
        goToRightSpikeBlue(-1);
        return;
    }

    private void locateTeamPropAndDropPixelRedSideFar() {
        //Drives to the middle point between all 3 spikes. Aligned with the center spike, and equidistant to
        //The 2 side spikes. Turns to the center and left to look for the team prop. Will go to whichever spike
        //it finds the prop on and heads to it to drop the spike. After dropping, it returns to the intial scanning
        //position
        goForwardStart(1);
        turnToHeading(0.5, 10);


        double curRunTime = runtime.time();
        //Looks for the prop on the center spike for 2 seconds
        while (runtime.time() < curRunTime + 2) {
            telemetry.addData("TFOD WORKING", "WAHOOOO");
            telemetryTfod("RedProp");

        }
        //If the prop was located on center spike, move towards it then drop pixels
        if (tempPropFound) {
            turnToHeading(0.5, -5);
            propLocation = 5;
            tagToDetect = 5;
            goToCenterSpikeBlue(1);
            waitTime(0.5);
            arm.openLeftClaw();
            waitTime(0.5);
            arm.setArmPivotPosition(150, 0.5);
            waitTime(0.5);
            goToCenterSpikeBlue(-1);
            return;
        }
        //Rotates 30 degrees left
        turnToHeading(0.5, 30);
        curRunTime = runtime.time();
        //Looks for the prop on the left spike for 2 seconds
        while (runtime.time() < curRunTime + 2) {
            telemetryTfod("RedProp");
            telemetry.addData("left", true);
            telemetry.update();
        }

        //If the prop was located on the left spike, move towards it then drop pixels
        if (tempPropFound && propLocation == 0) {
            propLocation = 4;
            tagToDetect = 4;
            turnToHeading(0.5, 15);
            goToLeftSpikeBlue(1);
            waitTime(0.5);
            arm.openLeftClaw();
            waitTime(0.5);
            arm.setArmPivotPosition(150, 0.5);
            waitTime(0.5);
            goToLeftSpikeBlue(-1);
            return;
        }
        propLocation = 6;
        tagToDetect = 6;
        turnToHeading(0.5, -25); //30
        goToRightSpikeBlue(1);
        turnToHeading(0.5, -50);
        waitTime(0.5);
        arm.openLeftClaw();
        waitTime(0.5);
        arm.setArmPivotPosition(170, 0.5);
        waitTime(1);
        turnToHeading(0.5, -25);
        goToRightSpikeBlue(-1);
        return;
    }

    private void locateTeamPropAndDropPixelRedSide() {
        //Drives to the middle point between all 3 spikes. Aligned with the center spike, and equidistant to
        //The 2 side spikes. Turns to the center and left to look for the team prop. Will go to whichever spike
        //it finds the prop on and heads to it to drop the spike. After dropping, it returns to the intial scanning
        //position in preparation for moving towards the board
        goForwardStart(1);
        turnToHeading(0.5, 20);


        double curRunTime = runtime.time();
        //Looks for the prop on the center spike for 2 seconds
        while(runtime.time() < curRunTime + 2) {
            telemetry.addData("TFOD WORKING", "WAHOOOO");
            telemetryTfod("RedProp");

        }
        //If the prop was located on center spike, move towards it then drop pixels
        if(tempPropFound) {
            propLocation = 5;
            tagToDetect = 5; //5
            turnToHeading(0.5, 0);
            goToCenterSpikeRed(1);
            waitTime(0.5);
            arm.openLeftClaw();
            waitTime(0.5);
            goToCenterSpikeRed(-1);
            return;
        }
        //Rotates 30 degrees left
        turnToHeading(0.5, 45);
        curRunTime = runtime.time();
        //Looks for the prop on the left spike for 2 seconds
        while (runtime.time() < curRunTime + 2) {
            telemetryTfod("RedProp");
            telemetry.addData("left", true);
            telemetry.update();
        }

        //If the prop was located on the left spike, move towards it then drop pixels
        if(tempPropFound && propLocation == 0) {
            propLocation = 4;
            tagToDetect = 4;//4
            turnToHeading(0.5, 35);
            goToLeftSpikeRed(1);
            waitTime(0.5);
            arm.openLeftClaw();
            waitTime(0.5);
            goToLeftSpikeRed(-1);
            return;
        }
        //Turns 60 degrees right (30 to -30)
        propLocation = 6; //6
        tagToDetect = 6; //6
        turnToHeading(0.5, -25);
        goToRightSpikeRed(1);
        waitTime(0.5);
        arm.openLeftClaw();
        waitTime(0.5);
        goToRightSpikeRed(-1);
        return;
    }


    private void goForwardStart(double direction) {
        //Approx 24 inches
        driveForwardDistance(0.3, 0.2*direction);
    }

    private void goToLeftSpikeBlue(double direction) {
        //Approx 2.5 inches
        //driveForwardDistance(0.5, 0.076);
        driveForwardDistance(0.5, 0.19 * direction);
    }

    private void goToLeftSpikeRed(double direction) {
        driveForwardDistance(0.5, 0.28 * direction);
    }

    private void goToRightSpikeBlue(double direction) {
        driveForwardDistance(0.5, 0.33 * direction);
    }

    private void goToRightSpikeRed(double direction) {driveForwardDistance(0.5, 0.27 * direction);}
    private void goToCenterSpikeBlue(double direction) {
        //Approx 6 inches
        driveForwardDistance(0.5, 0.39 * direction);
    }

    private void goToCenterSpikeRed(double direction) {
        driveForwardDistance(0.5, 0.4 * direction);
    }





    private void initTfodAndAprilTag() {

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


        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

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
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.70f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfodAndAprilTag()



    private void telemetryTfod(String propLabel) {
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
            if (recognition.getLabel() == propLabel) {
                tempPropFound = true;
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()



    private void telemetryAprilTag() {
        //Error null pointer exception on tagLocator
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private void moveToAprilTag(double desiredDist, int tag, double desiredStrafeOffset, double desiredHeadingOffset) {
        boolean targetFound = false;
        AprilTagDetection desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagToDetect < 0) || (detection.id == tag)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            rangeError      = (desiredTag.ftcPose.range - desiredDist);
            tagHeadingError    = desiredTag.ftcPose.bearing - desiredHeadingOffset;
            yawError        = desiredTag.ftcPose.yaw - desiredStrafeOffset;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double drive  = Range.clip(rangeError * P_SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(tagHeadingError * P_TAG_TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            double strafe = Range.clip(-yawError * P_STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            chassis.moveRobotMecanum(drive, strafe, turn);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else {
            chassis.moveRobotMecanum(0, 0, 0);
            telemetry.addData("TAG NOT FOUND", "TAG NOT FOUND");
        }
    }


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
        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ticksToDist = metersToEncoderTicks(distanceMeters, 1440, WHEEL_CIRCUMFERENCE_METERS);
        frontLeftDriveMotor.setTargetPosition(ticksToDist);
        frontRightDriveMotor.setTargetPosition(ticksToDist);
        backLeftDriveMotor.setTargetPosition(ticksToDist);
        backRightDriveMotor.setTargetPosition(ticksToDist);


        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.moveRobotMecanum(power, 0, 0);


        //Signals the desired distance has been achieved
        //Will stay in this loop until the encoders have reached the desired position
        while (frontLeftDriveMotor.isBusy() && frontRightDriveMotor.isBusy() && backRightDriveMotor.isBusy() && backLeftDriveMotor.isBusy()) {
            sendTelemetry();
        }
        chassis.moveRobotMecanum(0, 0, 0);
    }


    public void sendTelemetry() {
        telemetry.addData("dist", encoderToMeters(frontLeftDriveMotor.getCurrentPosition(), ENCODER_TICKS_TO_REVOLUTION, WHEEL_CIRCUMFERENCE_METERS));
        telemetry.addData("Heading", getHeading());
        telemetry.addData("Claw piv enc", clawPivotMotor.getCurrentPosition());
        telemetry.addData("Motor piv enc", armMotor.getCurrentPosition());
        telemetry.addData("Prop location", propLocation);
        telemetry.addData("PropFound?", tempPropFound);
        telemetry.addData("leftFront", frontLeftDriveMotor.getCurrentPosition());
        telemetry.addData("leftBack", backLeftDriveMotor.getCurrentPosition());
        telemetry.addData("rightFront", frontRightDriveMotor.getCurrentPosition());
        telemetry.addData("rightBack", backRightDriveMotor.getCurrentPosition());
        telemetry.update();
    }


    public void turnToHeading(double maxTurnSpeed, double heading) {
        driveOrTurn = "TURN";
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
            chassis.moveRobotMecanum(0, 0, turnSpeed);
        }
        // Stop all motion;
        chassis.moveRobotMecanum(0, 0, 0);
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

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    private void setAutoExposure(int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Auto) {
                exposureControl.setMode(ExposureControl.Mode.Auto);
                sleep(50);
            }
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
