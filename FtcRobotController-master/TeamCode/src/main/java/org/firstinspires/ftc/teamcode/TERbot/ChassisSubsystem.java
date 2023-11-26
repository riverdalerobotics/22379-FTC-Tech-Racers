package org.firstinspires.ftc.teamcode.TERbot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ChassisSubsystem {

    DcMotor leftMotor;

    DcMotor rightMotor;

    double driveSpeed;
    double turnSpeed;
    double leftSpeed;
    double rightSpeed;

    public ChassisSubsystem(DcMotor leftDrive, DcMotor rightDrive) {
        this.leftMotor = leftDrive;
        this.rightMotor = rightDrive;
    }


    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.


        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;


        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }


}
