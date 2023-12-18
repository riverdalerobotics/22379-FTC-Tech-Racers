package org.firstinspires.ftc.teamcode.TERbot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {



    class ArmConstants {
        //The arm starts at its lowest position for frame perimeter and set up, but it's min position
        //is higher up. This is because going below the desired min position will result in the claw
        //getting stuck under the chassis while driving
        public static final int MIN_ARM_POSITION = 100;
        public static final int MAX_ARM_POSITION = 250;

    }

    class ClawConstants {
        public static final double CLAW_SERVOS_START_POSITION = 0.95;
        public static final double MAX_CLAW_POSITION = 0.975;
        public static final double MIN_CLAW_POSITION = 0.9;

        //These positions are calibrated on the assumption that the 0 value is when
        //the claw is resting on the ground while the arm is as far down as it can go
        public static final int MAX_CLAW_PIVOT_POSITION = 100;
        public static final int MIN_CLAW_PIVOT_POSITION = -290;
    }


    class DriveConstants {

    }

    class DroneConstants {
        public static final double START_DRONE_SERVO_POS = 0;
        public static final double FIRE_DRONE_SERVO_POS = 0.88;
    }


    static class OdometryConstants {
        //Wheel circumference is in centimeters
        //Approx 0.314 meters
        public static final double WHEEL_CIRCUMFERENCE_METERS = 10 * Math.PI /100;

        public static final int ENCODER_TICKS_TO_REVOLUTION = 1440;

        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        public static final RevHubOrientationOnRobot REV_HUB_ORIENTATION = new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION);
    }
}
