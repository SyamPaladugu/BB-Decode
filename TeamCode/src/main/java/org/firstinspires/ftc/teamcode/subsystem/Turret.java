package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret implements Subsystem {

    private Servo turretServo1, turretServo2;
    private DcMotor shooterMotor1, shooterMotor2;

    private Telemetry telemetry;

    // Goal position
    private static final Vector2d GOAL_POS = new Vector2d(-62, -57.5);

    // Turret servo positions (adjust these based on your servo range)
    private static final double SERVO_HOME_POS = 0.5;
    private static final double SERVO_LEFT_POS = 0.0;   // Fully left
    private static final double SERVO_RIGHT_POS = 1.0;  // Fully right

    // Shooter settings
    private static final double SHOOTER_POWER = 1.0;
    private static final long SHOOT_DURATION_MS = 500;

    // Current state
    private Vector2d robotPos = new Vector2d(0, 0);
    private double robotHeading = 0;
    private boolean isAligning = false;
    private double currentTurretAngle = 0;

    public Turret(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretServo1 = map.get(Servo.class, "turret_servo_1");
        turretServo2 = map.get(Servo.class, "turret_servo_2");
        shooterMotor1 = map.get(DcMotor.class, "shooter_motor_1");
        shooterMotor2 = map.get(DcMotor.class, "shooter_motor_2");

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() {
        telemetry.addData("Turret", "Initialized");
        telemetry.update();
    }

    // update robot pos from rr odo
    public void updatePose(Vector2d pos, double headingDeg) {
        this.robotPos = pos;
        this.robotHeading = headingDeg;
    }

    //calculate angle with trig
    private double calculateTurretAngle(Vector2d robotPos, double robotHeadingDeg, Vector2d goalPos) {
        // Calculate vector from robot to goal
        Vector2d toGoal = goalPos.minus(robotPos);

        // Calculate absolute angle to goal using atan2
        double angleToGoal = Math.toDegrees(Math.atan2(toGoal.y, toGoal.x));

        // Normalize to 0-360 range
        if (angleToGoal < 0) {
            angleToGoal += 360;
        }

        // Normalize robot heading to 0-360 range
        double normalizedHeading = robotHeadingDeg;
        if (normalizedHeading < 0) {
            normalizedHeading += 360;
        }

        // Calculate the difference between goal angle and robot heading
        double turretAngle = angleToGoal - normalizedHeading;

        // Normalize turret angle to -180 to 180 range (shortest rotation path)
        if (turretAngle > 180) {
            turretAngle -= 360;
        }
        if (turretAngle < -180) {
            turretAngle += 360;
        }

        return turretAngle;
    }

    // converts angle to servo pos
    //Sets position
    private void setTurretPosition(double turretAngleDeg) {
        // Map angle to servo position (adjust range based on your turret's limits)
        // -90 to +90 degrees typically maps to 0.0 to 1.0 servo position
        double servoPos = 0.5 + (turretAngleDeg / 180.0) * 0.5;

        // Clamp servo position to 0.0 - 1.0 range
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(1.0 - servoPos); // Opposite servo
    }

    // return turret to home pos.
    private void returnTurretHome() {
        turretServo1.setPosition(SERVO_HOME_POS);
        turretServo2.setPosition(1.0 - SERVO_HOME_POS);
        currentTurretAngle = 0;
    }

    //runs both shootermotors: auto shoot.
    private void runShooters(double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    // Shoots balls pretty self explanatory
    public void shootBall() {
        new Thread(() -> {
            runShooters(SHOOTER_POWER);
            try {
                Thread.sleep(SHOOT_DURATION_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            runShooters(0);
        }).start();
    }

   // i think this stops the shooters...
    public void stopShooter() {
        runShooters(0);
    }

    @Override
    public void update() {
        if (isAligning) {
            double turretAngle = calculateTurretAngle(robotPos, robotHeading, GOAL_POS);
            setTurretPosition(turretAngle);
            currentTurretAngle = turretAngle;
        }

        telemetry.addData("Robot Pos", "X: %.2f, Y: %.2f", robotPos.x, robotPos.y);
        telemetry.addData("Robot Heading (deg)", robotHeading);
        telemetry.addData("Goal Pos", "X: %.2f, Y: %.2f", GOAL_POS.x, GOAL_POS.y);
        telemetry.addData("Turret Angle (deg)", currentTurretAngle);
        telemetry.addData("Turret Aligning", isAligning);
        telemetry.addData("Servo 1 Pos", turretServo1.getPosition());
        telemetry.addData("Servo 2 Pos", turretServo2.getPosition());
    }

    @Override
    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
        // Hold right bumper to align turret to goal
        if (gp1.right_bumper) {
            isAligning = true;
        } else {
            isAligning = false;
            returnTurretHome();
        }

        // Press A to shoot
        if (gp1.a) {
            shootBall();
        }

        // Manual shooter control
        if (gp1.x) {
            runShooters(SHOOTER_POWER);
        } else if (gp1.y) {
            runShooters(-SHOOTER_POWER);
        } else {
            runShooters(0);
        }
    }

    public boolean isAligning() {
        return isAligning;
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
}