package org.firstinspires.ftc.teamcode.TERbot;

import androidx.appcompat.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TERbot.Constants.*;


public class ArmSubsystem {

    DcMotor armPivotMotor;
    DcMotor clawPivotMotor;
    Servo rightClawServo;
    Servo leftClawServo;

    int maxArmPivotPosition = Constants.ArmConstants.MAX_ARM_POSITION;
    int minArmPivotPosition = Constants.ArmConstants.MIN_ARM_POSITION;
    int armPivotMotorPosition;


    int maxClawPivotPosition = Constants.ClawConstants.MAX_CLAW_PIVOT_POSITION;
    int minClawPivotPosition = Constants.ClawConstants.MIN_CLAW_PIVOT_POSITION;
    int clawPivotMotorPosition;


    double maxClawServosPosition = Constants.ClawConstants.MAX_CLAW_POSITION;
    double minClawServosPosition = Constants.ClawConstants.MIN_CLAW_POSITION;
    double clawServosDefaultPosition = Constants.ClawConstants.CLAW_SERVOS_START_POSITION;
    double leftClawServoCurrentPosition = clawServosDefaultPosition;
    double rightClawServoCurrentPosition = clawServosDefaultPosition;




    public ArmSubsystem(DcMotor armPiv, Servo leftClawServo, Servo rightClawServo, DcMotor clawPiv) {
        this.armPivotMotor = armPiv;
        this.clawPivotMotor = clawPiv;
        this.rightClawServo = rightClawServo;
        this.leftClawServo = leftClawServo;

        //These should always be 0
        this.clawPivotMotorPosition = this.clawPivotMotor.getCurrentPosition();
        this.armPivotMotorPosition = this.armPivotMotor.getCurrentPosition();

        this.armPivotMotor.setDirection((DcMotorSimple.Direction.FORWARD));
        this.leftClawServo.setDirection(Servo.Direction.FORWARD);
        this.rightClawServo.setDirection(Servo.Direction.REVERSE);
        this.clawPivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        clawPivotMotor.setTargetPosition(maxClawPivotPosition);
        clawPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armPivotMotor.setTargetPosition(minArmPivotPosition);
        armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void closeClaw() {
        this.setClawPosition(ClawConstants.MAX_CLAW_POSITION);
    }

    public void openClaw() {
        this.setClawPosition(ClawConstants.MIN_CLAW_POSITION);
    }

    public void closeLeftClaw() {this.setLeftClawPosition(ClawConstants.MAX_CLAW_POSITION);}
    public void openLeftClaw() {this.setLeftClawPosition(ClawConstants.MIN_CLAW_POSITION);}
    public void closeRightClaw() {this.setRightClawPosition(ClawConstants.MAX_CLAW_POSITION);}
    public void openRightClaw() {this.setRightClawPosition(ClawConstants.MIN_CLAW_POSITION);}





    //TELEOP//
    //These methods incrementally increase the pivot values to be controller-responsive
    //TELEOP//
    //TODO: Check to see what happens when you pass in a controller axis and add to an integer
    public void pivotClaw(double clawPivotInput, double power) {
        clawPivotMotorPosition += clawPivotInput;
        clawPivotMotorPosition = HelperMethods.clipInteger(clawPivotMotorPosition, minClawPivotPosition, maxClawPivotPosition);
        clawPivotMotor.setPower(power); //0.4 (0.6 for auto)
        clawPivotMotor.setTargetPosition(clawPivotMotorPosition);
    }
    public void pivotArm(double armPivotInput, double power) {
        armPivotMotorPosition += armPivotInput;
        armPivotMotorPosition = HelperMethods.clipInteger(armPivotMotorPosition, minArmPivotPosition, maxArmPivotPosition);
        armPivotMotor.setPower(power);
        armPivotMotor.setTargetPosition(armPivotMotorPosition);
    }
    public void moveClaw(double clawInput) {
        //Input is divided by 80 to allow for more precise claw movement
        rightClawServoCurrentPosition += clawInput / 80;
        leftClawServoCurrentPosition += clawInput / 80;
        rightClawServoCurrentPosition = HelperMethods.clipDouble(rightClawServoCurrentPosition, minClawServosPosition, maxClawServosPosition);
        leftClawServoCurrentPosition = HelperMethods.clipDouble(leftClawServoCurrentPosition, minClawServosPosition, maxClawServosPosition);
        rightClawServo.setPosition(rightClawServoCurrentPosition);
        leftClawServo.setPosition(leftClawServoCurrentPosition);
    }




    //AUTONOMOUS//
    //These methods can be set to exact positions for ease of use and accuracy
    //AUTONOMOUS//
    public void setClawPivotPosition(int clawPivPos, double power) {
        clawPivotMotorPosition = HelperMethods.clipInteger(clawPivPos, minClawPivotPosition, maxClawPivotPosition);
        clawPivotMotor.setPower(power);
        clawPivotMotor.setTargetPosition(clawPivotMotorPosition);
    }
    public void setArmPivotPosition(int armPivPos, double power) {
        armPivotMotorPosition = HelperMethods.clipInteger(armPivPos, minArmPivotPosition, maxArmPivotPosition);
        armPivotMotor.setPower(power);
        armPivotMotor.setTargetPosition(armPivotMotorPosition);
    }
    public void setClawPosition(double clawPos) {
        //Input is divided by 80 to allow for more precise claw movement
        leftClawServoCurrentPosition = HelperMethods.clipDouble(clawPos, minClawServosPosition, maxClawServosPosition);
        rightClawServoCurrentPosition = HelperMethods.clipDouble(clawPos, minClawServosPosition, maxClawServosPosition);
        rightClawServo.setPosition(rightClawServoCurrentPosition);
        leftClawServo.setPosition(leftClawServoCurrentPosition);
    }


    //These methods should only be used to toggle the left and right claws open and close
    //They do not actually change the value of the claw's current position. This means
    //tapping the triggers will immediately return the claw to its position before the button was pressed
    public void setLeftClawPosition(double leftClawPos) {
        leftClawServoCurrentPosition = leftClawPos;
        leftClawServoCurrentPosition = HelperMethods.clipDouble(leftClawServoCurrentPosition, minClawServosPosition, maxClawServosPosition);
        leftClawServo.setPosition(leftClawPos);
    }

    public void setRightClawPosition(double rightClawPos) {
        rightClawServoCurrentPosition = rightClawPos;
        rightClawServoCurrentPosition = HelperMethods.clipDouble(rightClawServoCurrentPosition, minClawServosPosition, maxClawServosPosition);
        rightClawServo.setPosition(rightClawPos);
    }
}
