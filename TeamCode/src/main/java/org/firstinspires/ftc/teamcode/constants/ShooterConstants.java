package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double closeShootRPM = 2200;// 1800
    public static double farShootRPM = 3000; //2.16k
    public static double shootingMultiplier = 3;
    public static double kf = 0.2, kp = 0.3, ki = 0, kd = 0.00012;
    public static double tuningTestingRPM = 0;
    public static double tuningTestingCounterRollerRPM = 0;
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;
    public static double CRkf = 0.3, CRkp = 0.004, CRki = 0, CRkd = 0.00001;
    public static int CR_TICKS_PER_REV = 28;
    public static int RPM_OFFSET = -50;



}
