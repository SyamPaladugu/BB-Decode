package org.firstinspires.ftc.teamcode;


//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class Shooter {
//
//    // Constants
//    private final double GRAVITY = 9.81;   // gravity lol
//    private double FLYWHEEL_RADIUS = 0.039; //flywheel radius (78mm diameter)
//    private final double MAX_RPM = 6000.0; // motor’s maximum speed
//    private final double EF = 0.75; // efficiency factor need to tune
//    private final double SHOOTER_HEIGHT = 0.12;   // height of shooter center (meters)
//    private final double GOAL_HEIGHT = 0.381;// height of target goal center (meters)
//
//
//    // --- Dynamic variables
//    private double distance;        // distance to goal (meters)
//    private  double launchAngle;// launch angle in radians (servo-controlled)
//    private double rpm;             // computed target flywheel RPM
//    private double power;
//    TrackAprilTag tag;
//
//    //Mechanisms
//    private DcMotor shooter;
//    HardwareMap hardwareMap;
//    Telemetry telemetry;
//
//    public Shooter(HardwareMap hardwareMap ){
//         this.hardwareMap= hardwareMap;
//         this.telemetry = telemetry;
//    }
//    public double getPower(){
//        distance = tag.stuff()[0];
//        launchAngle = Math.toRadians(launchAngle);
//        double h = GOAL_HEIGHT - SHOOTER_HEIGHT;
//
//        //  required launch velocity
//        double numerator = distance / Math.cos(launchAngle);
//        double denom = 2 * (distance * Math.tan(launchAngle) - h);
//        double v = numerator * Math.sqrt(GRAVITY/ denom);
//
//        //  flywheel RPM and motor power
//        rpm = (30 * v) / (Math.PI * EF * FLYWHEEL_RADIUS);
//        power = rpm / MAX_RPM;
//        return power;
//    }
//
//
//    public void shoot(){
//        shooter.setPower(getPower());
//    }
//
//
//
//
//}
