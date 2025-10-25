package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class outtake {
    private double power;
    private DcMotorEx bare;
    private Servo hoodServo;
    Telemetry telemetry;
    private static final double FLYWHEEL_RPM = 6000.0;
    private static final double WHEEL_DIAMETER_MM = 72.0;
    private static final double SHOOTER_HEIGHT_INCHES = 18.0;  // MEASURE YOUR ROBOT
    private static final double TARGET_HEIGHT_INCHES = 36.0;   // MEASURE YOUR FIELD
    private static final double VELOCITY_EFFICIENCY = 0.85;     // Tune based on testing (0.7-0.9)

    // Physics constants
    private static final double GRAVITY = 9.81; // m/s²
    private static final double INCHES_TO_METERS = 0.0254;

    // Calculated launch velocity
    private final double launchVelocity = 0;
    private final double heightDifference = 0;

    public outtake(HardwareMap map){
        bare = map.get(DcMotorEx.class,"outtake");
        hoodServo = map.get(Servo.class,"HoodServo");
    }

    //distanceInches Horizontal distance to target in inches
    //useHighArc True for high arc shot, false for low arc
    //θ = arctan((v₀² + √(v₀⁴ - g(gd² + 2v₀²h))) / (gd))
    //Where:
    //* v₀ = 22.6 m/s (or tune this based on testing)
    //* g = 9.81 m/s²
    //* d = horizontal distance to target (from camera/odometry)
    //* h = target height - shooter height (in meters)
    //* θ = hood angle in radians (convert to degrees)
    public double calculateHoodAngle(double distanceInches){
        double d = distanceInches * INCHES_TO_METERS;
        double h = heightDifference;
        double v = launchVelocity;
        double g = GRAVITY;

        double discriminant = Math.pow(v, 4) - g * (g * d * d + 2 * v * v * h);

        if (discriminant < 0) {
            return -1;
        }

        double launchAngle =
                Math.atan(Math.max(v,2) + Math.sqrt(Math.max(v,4)-g*(g*Math.max(d,2)+2*Math.max(v,2)*h))/(g*d));


        double servoPosition = (63.2 - launchAngle) / (63.2 - 47.5);


        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        return servoPosition;
    }

    public void shoot(double power){
        bare.setPower(power);
    }



}
