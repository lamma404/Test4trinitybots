package org.firstinspires.ftc.teamcode.elevator;

public class ElevatorConstants {

    public static double MAX_HEIGHT = 26;
    public static double MIN_HEIGHT = 0;

    public static double TICKS_PER_REV = 384.5; //Gobilda 5203 motor, 435 RPM, Encoder Resolution	384.5 PPR at the Output Shaft
    public static double WHEEL_RADIUS = 0.74803149; // in inch, pulley 38mm diameter

    public static final int MAX_VELOCITY=2600; // 23 in ticks, this should be from the result in Elev_1_MaxVelocityTest.java
    // MAX_VELOCITY = MAX CPS = (RPM at 12V) * (Encoder counts per Rev)/60 = 1150 * 145.1 / 60 = 2781.0833 (CPS: Counts per second)
    // F = 32767/2781.0833 = 11.7820

    public static final double NEW_F = 14.434801762114537; //32767/MAX_VELOCITY;
    public static final double NEW_P = 1.606*4; //0.1 * NEW_F;
    public static final double NEW_I = 0.1606*5; //0.1 * NEW_P;
    public static final double NEW_D = 0;
    public static final double NEW_P_POSITION = 10.0;

    public static double encoderTicksToInches(double ticks) {
        return (WHEEL_RADIUS * 2 * Math.PI * ticks) / TICKS_PER_REV;
    }

    public static int encoderInchesToTicks(double inches) {
        return (int)((inches * TICKS_PER_REV)/(WHEEL_RADIUS * 2 * Math.PI));
    }
}
