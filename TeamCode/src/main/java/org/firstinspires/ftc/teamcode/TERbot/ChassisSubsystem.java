package org.firstinspires.ftc.teamcode.TERbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ChassisSubsystem {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    double fwdSpeed;
    double strafeSpeed;
    double turnSpeed;
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;


    public ChassisSubsystem(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive) {
        this.frontLeftMotor = frontLeftDrive;
        this.frontRightMotor = frontRightDrive;
        this.backRightMotor = backRightDrive;
        this.backLeftMotor = backLeftDrive;

        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    //Forward is positive x
    //Left is positive y
    //CCW is positive yaw
    public void moveRobotMecanum(double fwd, double strafe, double turn) {
        fwdSpeed = fwd;
        strafeSpeed = strafe;
        turnSpeed = turn;

        frontLeftSpeed = fwdSpeed - strafeSpeed - turnSpeed;
        backLeftSpeed = fwdSpeed + strafeSpeed - turnSpeed;
        frontRightSpeed = fwdSpeed + strafeSpeed + turnSpeed;
        backRightSpeed= fwdSpeed -strafeSpeed + turnSpeed;

        double max = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
        max = Math.max(max, Math.abs(backLeftSpeed));
        max = Math.max(max, Math.abs(backRightSpeed));

        if(max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }
        frontLeftMotor.setPower(frontLeftSpeed);
        frontRightMotor.setPower(frontRightSpeed);
        backLeftMotor.setPower(backLeftSpeed);
        backRightMotor.setPower(backRightSpeed);
    }
}
