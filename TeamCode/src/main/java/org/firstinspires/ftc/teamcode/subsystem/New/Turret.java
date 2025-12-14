package org.firstinspires.ftc.teamcode.subsystem.New;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Turret implements Subsystem {

    private Servo turretServo1, turretServo2;

    private Telemetry telemetry;

    public static double GOAL_X = -60;
    public static double GOAL_Y = -60;

    private static final double SERVO_HOME_POS = 0.5;

    // Current state
    private Vector2d robotPos = new Vector2d(0, 0);
    private double robotHeading = 0;
    public boolean isAligning = false;
    private double currentTurretAngle = 0;
    private double currentDistanceToGoal = 0;

    private Vector2d turretPos = new Vector2d(0,0);

    private static double turretOffsetInchesx = -5;
    private static double turretOffsetInchesy = -2;



    public Turret(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretServo1 = map.get(Servo.class, "turret_servo_1");
        turretServo2 = map.get(Servo.class, "turret_servo_2");
    }

    @Override
    public void init() {
        telemetry.addData("Turret", "Initialized");
        telemetry.update();
    }


    public void updatePose(Vector2d pos, double headingDeg) {
        this.robotPos = pos;
        this.robotHeading = headingDeg;

        turretPos = new Vector2d(
                robotPos.x - turretOffsetInchesx * Math.cos(robotHeading),
                robotPos.y - turretOffsetInchesy * Math.sin(robotHeading)
        );
    }


    private double calculateDistanceToGoal(Vector2d turretPos, Vector2d goalPos) {
        Vector2d toGoal = goalPos.minus(turretPos);
        return Math.sqrt(toGoal.x * toGoal.x + toGoal.y * toGoal.y);
    }
    private double calculateTurretAngle(Vector2d robotPos, double robotHeadingDeg, Vector2d goalPos) {
        Vector2d toGoal = goalPos.minus(robotPos);
        double angleToGoal = Math.toDegrees(Math.atan2(toGoal.y, toGoal.x));
        angleToGoal = AngleUnit.normalizeDegrees(angleToGoal) + 180;
//        if (angleToGoal < 0) {
//            angleToGoal += 360;
//        }
//        double normalizedHeading = robotHeadingDeg;
//        if (normalizedHeading < 0) {
//            normalizedHeading += 360;
//        }
        double turretAngle = AngleUnit.normalizeDegrees(angleToGoal - robotHeadingDeg);
//        while (turretAngle > 180) {
//            turretAngle -= 360;
//        }
//        while (turretAngle < -180) {
//            turretAngle += 360;
//        }
        return turretAngle;
    }
    private void setTurretPosition(double turretAngleDeg) {
        double servoPos = 0.5 + (turretAngleDeg / 270.0);
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));
        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
    }

    public void returnTurretHome() {
        turretServo1.setPosition(SERVO_HOME_POS);
        turretServo2.setPosition(SERVO_HOME_POS);
        currentTurretAngle = 0;
    }

    @Override
    public void update() {
        Vector2d goalPos = new Vector2d(GOAL_X, GOAL_Y);
        currentDistanceToGoal = calculateDistanceToGoal(turretPos, goalPos);

        if (isAligning) {
            double turretAngle = calculateTurretAngle(robotPos, robotHeading, goalPos);
            setTurretPosition(turretAngle);
            currentTurretAngle = turretAngle;
        }

        telemetry.addData("Goal Position", "X: %.2f, Y: %.2f", GOAL_X, GOAL_Y);
        telemetry.addData("Distance to Goal (in)", "%.2f", currentDistanceToGoal);
        telemetry.addData("Turret Angle (deg)", currentTurretAngle);
        telemetry.addData("Turret Aligning", isAligning);
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        if (gp1.left_bumper) {
            isAligning = true;
        } else {
            isAligning = false;
            returnTurretHome();
        }
    }

    public boolean isAligning() {
        return isAligning;
    }

    public void setAligning(boolean aligning) {
        this.isAligning = aligning;
    }

    public double getCurrentTurretAngle() {
        return currentTurretAngle;
    }

    public Vector2d getRobotPosition() {
        return robotPos;
    }

    public double getRobotHeading() {
        return robotHeading;
    }

    public double getDistanceToGoal() {
        return currentDistanceToGoal;
    }
}