package org.firstinspires.ftc.teamcode;

public class Constants {

    public static final int TARGET_LEVEL_TOP = 3;
    public static final int TARGET_LEVEL_MIDDLE = 2;
    public static final int TARGET_LEVEL_BOTTOM = 1;
    public static final int TARGET_LEVEL_DEFAULT = TARGET_LEVEL_TOP;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    ;
}
