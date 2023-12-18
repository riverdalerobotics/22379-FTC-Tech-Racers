package org.firstinspires.ftc.teamcode.TERbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TERbot.Constants.*;

import java.util.List;

@TeleOp(name="Aidan Drive", group="Linear OpMode")

public class PrimaryTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor frontLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor backLeftDriveMotor;
    private DcMotor backRightDriveMotor;
    private DcMotor armMotor;

    private DcMotor clawPivotMotor;

    private Servo rightClawServo;
    private Servo leftClawServo;

    private Servo droneServo;

    public WebcamName camera;

    AprilTagLocation tagLocator = new AprilTagLocation();
    private int encoderTickRatio = OdometryConstants.ENCODER_TICKS_TO_REVOLUTION;


    ChassisSubsystem chassis;
    ArmSubsystem arm;

    public void runOpMode() {

        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        tagLocator.initAprilTag(camera);
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        clawPivotMotor = hardwareMap.get(DcMotor.class, "clawPivotMotor");
        droneServo = hardwareMap.get(Servo.class, "droneServo");




        telemetry.addData("Status", "Initialized");
        telemetry.addData("ArmEncoder", armMotor.getCurrentPosition());
        telemetry.update();


        chassis = new ChassisSubsystem(frontLeftDriveMotor, frontRightDriveMotor, backLeftDriveMotor, backRightDriveMotor);
        arm = new ArmSubsystem(armMotor, leftClawServo, rightClawServo, clawPivotMotor);


        waitForStart();
        runtime.reset();


        //These receive pure controller inputs
        double fwdPwr;
        double strafePwr;
        double rotationPwr;
        double armPwr;
        double clawServoInput;
        double clawPivotInput;
        double droneServoPos = DroneConstants.START_DRONE_SERVO_POS;


        while (opModeIsActive()) {
            fwdPwr = -gamepad1.left_stick_y;
            strafePwr = -gamepad1.left_stick_x;
            rotationPwr = -gamepad1.right_stick_x;
            armPwr = -gamepad2.left_stick_y;
            clawServoInput = gamepad2.right_trigger - gamepad2.left_trigger;
            clawPivotInput = -gamepad2.right_stick_y * 1.5;

            if (gamepad2.a) {
                arm.closeRightClaw();
            }

            if (gamepad2.b) {
                arm.openRightClaw();
            }

            if (gamepad2.x) {
                arm.closeLeftClaw();
            }

            if (gamepad2.y) {
                arm.openLeftClaw();
            }

            if (gamepad1.a) {
                droneServoPos = DroneConstants.FIRE_DRONE_SERVO_POS;
            }
            else if (gamepad1.b) {
                droneServoPos = DroneConstants.START_DRONE_SERVO_POS;
            }
            droneServo.setPosition(droneServoPos);



            //Input is divided by 80 in method to allow for more precise claw movement
            arm.moveClaw(clawServoInput);

            arm.pivotClaw(clawPivotInput, 0.4);
            arm.pivotArm(armPwr, 0.4);
            chassis.moveRobotMecanum(fwdPwr, strafePwr, rotationPwr);

            telemetryAprilTag();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                tagLocator.visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                tagLocator.visionPortal.resumeStreaming();
            }

            telemetry.addData("Left stick x", strafePwr);
            telemetry.addData("left stick y", fwdPwr);
            telemetry.addData("right stick x", rotationPwr);
            telemetry.addData("FrontLeft encoder", frontLeftDriveMotor.getCurrentPosition());
            telemetry.addData("BackLeft encoder", backLeftDriveMotor.getCurrentPosition());
            telemetry.addData("FrontRigght encoder", frontRightDriveMotor.getCurrentPosition());
            telemetry.addData("BackRight enc", backRightDriveMotor.getCurrentPosition());
            telemetry.addData("armMotorEncoder", armMotor.getCurrentPosition());
            telemetry.addData("DesiredPosition", arm.armPivotMotorPosition);
            telemetry.addData("Left claw servo POS", leftClawServo.getPosition());
            telemetry.addData("Right claw servo POS", rightClawServo.getPosition());
            telemetry.addData("Claw pivot encoder", clawPivotMotor.getCurrentPosition());


            telemetry.update();


        }
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = tagLocator.aprilTag.getDetections();
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
}
