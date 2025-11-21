package org.firstinspires.ftc.teamcode.opMode.teleOp;

import android.text.TextUtils;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="TestTele", group = " ")
public class
MecanumTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotor intakeMotor;
    private DcMotorEx outtake;

    private Servo kicker;
    private CRServo transfer;
    private CRServo transfer2;
    private Servo hood;
    private double power;
    boolean toggleshooter = false;
    boolean togglekicker = false;
    boolean toggletransfer = false;
    double speed;
    private double pos;
    double hoodpos;
    boolean kickerpos;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "LFM");
        backLeftDrive = hardwareMap.get(DcMotor.class, "LBM");
        frontRightDrive = hardwareMap.get(DcMotor.class, "RFM");
        backRightDrive = hardwareMap.get(DcMotor.class, "RBM");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        hood = hardwareMap.get(Servo.class, "hood");
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            double rollers = gamepad1.right_trigger;
            double intake = gamepad1.right_trigger;
            intakeMotor.setPower(gamepad1.right_trigger);
            intakeMotor.setPower(-gamepad1.left_trigger);
            if (gamepad1.left_bumper) {

                if (gamepad1.yWasPressed()) {
                    kickerpos = false;
                    kicker.setPosition(0);
                }
                if (gamepad1.aWasPressed()) {
                    kickerpos = true;
                    kicker.setPosition(0.3);
                }
                if (gamepad1.dpadRightWasPressed()) {
                    toggletransfer = !toggletransfer;
                    if (toggletransfer) {
                        transfer.setPower(-1);
                    } else {
                        transfer.setPower(0);
                    }
                }

                if (gamepad1.rightBumperWasPressed()) {
                    toggleshooter = !toggleshooter;
                    if (toggleshooter) {
                        speed = 1;
                    } else {
                        speed = 0;
                    }
                }
            } else {
                if (gamepad2.rightBumperWasPressed()) {
                    toggleshooter = !toggleshooter;
                    if (toggleshooter) {
                        speed = 1;
                    } else {
                        speed = 0;
                    }
                }
                if (gamepad2.dpadUpWasPressed()) {
                    kickerpos = false;
                    kicker.setPosition(0);
                }
                if (gamepad2.dpadDownWasPressed()) {
                    kickerpos = true;
                    kicker.setPosition(0.3);
                }

                if (gamepad2.dpadRightWasPressed()) {
                    toggletransfer = !toggletransfer;
                    if (toggletransfer) {
                        transfer.setPower(-1);
                    } else {
                        transfer.setPower(0);
                    }
                    if (gamepad1.dpadUpWasPressed()) {
                        hoodpos += 0.05;
                        if (hoodpos >= 1) {
                            hoodpos = 1;
                        }

                    }

                    if (gamepad1.dpadDownWasPressed()) {
                        hoodpos -= 0.05;
                        if (hoodpos <= 0) {
                            hoodpos = 0;
                        }
                    }


                }
                outtake.setPower(speed);
                telemetry.addData("Power", outtake.getVelocity());


                hood.setPosition(hoodpos);
                telemetry.addLine();
                telemetry.addData("Hood Pos:", hoodpos);
                telemetry.addLine();
                telemetry.addData("Kicker state", kickerpos ? "Down" : "Up");
                telemetry.update();
            }
        }
    }
}