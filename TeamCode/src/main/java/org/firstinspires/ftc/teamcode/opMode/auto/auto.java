package org.firstinspires.ftc.teamcode.opMode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Drive Forward ", group = "Auto")
public class auto extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Constants for movement
    static final double COUNTS_PER_MOTOR_REV = 537.7; // need to change for our motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;   //
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // Typical wheel size
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double INCHES_PER_FOOT = 12.0;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "LFM");
        backLeftDrive = hardwareMap.get(DcMotor.class, "LBM");
        frontRightDrive = hardwareMap.get(DcMotor.class, "RFM");
        backRightDrive = hardwareMap.get(DcMotor.class, "RBM");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            driveForward(12, 0.5);  // move forward 12 inches (1 foot) at 50% speed
        }
    }

    private void driveForward(double inches, double power) {
        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        int newFLTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
        int newFRTarget = frontRightDrive.getCurrentPosition() + moveCounts;
        int newBLTarget = backLeftDrive.getCurrentPosition() + moveCounts;
        int newBRTarget = backRightDrive.getCurrentPosition() + moveCounts;

        frontLeftDrive.setTargetPosition(newFLTarget);
       frontRightDrive.setTargetPosition(newFRTarget);
        backLeftDrive.setTargetPosition(newBLTarget);
        backRightDrive.setTargetPosition(newBRTarget);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);

        while (opModeIsActive() &&
                (frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                        backLeftDrive.isBusy() && backRightDrive.isBusy())) {
            telemetry.addData("Driving", "");
            telemetry.update();
        }


        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
