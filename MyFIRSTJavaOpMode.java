package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Template: 2024 Brie 2", group="Linear Opmode")
public class MyFIRSTJavaOpMode extends LinearOpMode {
    // drive stuff
    private DcMotor lfd;
    private DcMotor lrd;
    private DcMotor rfd;
    private DcMotor rrd;
    private int lMod = -1; // changes CW to CCW for left drive motors
    private int rMod = 1; // changes CW to CCW for right drive motors
    // lift stuff
    private DcMotor lfl;
    private DcMotor lrl;
    private DcMotor rfl;
    private DcMotor rrl;
    private int lflStart;
    private int lrlStart;
    private int rflStart;
    private int rrlStart;
    private int lflDistance;
    private int lrlDistance;
    private int rflDistance;
    private int rrlDistance;
    private double maxChangePower = 0.2;
    private int goodEnoughDistance = 100;
    private int maxPowerDistance = 500;
    // plane stuff
    private Servo planeServo;
    private double planeStart = 0.5;
    private double planeDiff = 0.5;
    // claw stuff
    private Servo lClaw;
    private Servo rClaw;
    private double leftOpenPos = /*(double) 1/2*/ 0.703125;
    private double leftDiff = (double) -3/8;
    private double rightOpenPos = /*(double) 37/128*/ 0.296875;
    private double rightDiff = (double) 3/8;
    // drum stuff

    @Override
    public void runOpMode() {
        lfd = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        lrd = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rfd = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rrd = hardwareMap.get(DcMotor.class, "rightRearDrive");
/*
        lfl = hardwareMap.get(DcMotor.class, "leftFrontLift");
        lrl = hardwareMap.get(DcMotor.class, "leftRearLift");
        rfl = hardwareMap.get(DcMotor.class, "rightFrontLift");
        rrl = hardwareMap.get(DcMotor.class, "rightRearLift");

        lflStart = lfl.getCurrentPosition();
        lrlStart = lrl.getCurrentPosition();
        rflStart = rfl.getCurrentPosition();
        rrlStart = rrl.getCurrentPosition();

        lflDistance = 0;
        lrlDistance = 0;
        rflDistance = 0;
        rrlDistance = 0;

        planeServo = hardwareMap.get(Servo.class, "planeServo");
        planeServo.setPosition(planeStart);

        lClaw = hardwareMap.get(Servo.class, "leftClaw");
        rClaw = hardwareMap.get(Servo.class, "rightClaw");
*/

        boolean leftPressed = false;
        boolean rightPressed = false;
        boolean leftClosed = false;
        boolean rightClosed = false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("leftRearPosition", lrd.getCurrentPosition());
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y * -1;
            double rightY = gamepad1.right_stick_y * -1;
            telemetry.addData("leftX", "" + leftX);
            telemetry.addData("leftY", "" + leftY);
            telemetry.addData("rightY", "" + rightY);
            if (leftX == 0.0 && leftY == 0.0) {
                double rDepth = gamepad1.right_trigger;
                double lDepth = gamepad1.left_trigger;
                rotate(lDepth, rDepth);
            } else {
                translate(leftX, leftY);
            }
            /*if (gamepad1.left_bumper && !leftPressed) {
                leftClosed = !leftClosed;
                leftPressed = true;
            } else if (!gamepad1.left_bumper){
                leftPressed = false;
            }
            setClaw(true, leftClosed);
            if (gamepad1.right_bumper && !rightPressed) {
                rightClosed = !rightClosed;
                rightPressed = true;
            } else if (!gamepad1.right_bumper){
                rightPressed = false;
            }
            setClaw(false, rightClosed);*/
            //lift(rightY);
            //updateLiftDistances();
            telemetry.update();
        }

    }
    public double pythag(double num1, double num2) {
        return Math.sqrt(Math.pow(num1, 2) + Math.pow(num2, 2));
    }
    public void translate(double xVal, double yVal) {
        double totalPower = pythag(xVal, yVal);
        if (totalPower > 1) totalPower = 1;
        if (totalPower < -1) totalPower = -1;
        if (xVal == 0.0 && yVal == 0.0) {
            lfd.setPower(0.0);
            lrd.setPower(0.0);
            rfd.setPower(0.0);
            rrd.setPower(0.0);
        } else if (xVal >= 0.0 && yVal >= 0.0) {
            lrd.setPower(lMod * totalPower * -1.0);
            rfd.setPower(rMod * totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(lMod * oppPower);
            rrd.setPower(rMod * oppPower);
        } else if (xVal <= 0.0 && yVal >= 0.0) {
            lfd.setPower(lMod * totalPower);
            rrd.setPower(rMod * totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(lMod * oppPower * -1.0);
            rfd.setPower(rMod * oppPower);
        } else if (xVal <= 0.0 && yVal <= 0.0) {
            lrd.setPower(-1 * lMod * totalPower * -1.0);
            rfd.setPower(-1 * rMod * totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(lMod * oppPower);
            rrd.setPower(rMod * oppPower);
        } else {
            lfd.setPower(-1 * lMod * totalPower);
            rrd.setPower(-1 * rMod * totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(lMod * oppPower * -1.0);
            rfd.setPower(rMod * oppPower);
        }
    }
    public void lift(double power) {
        double lflMod = changeFromDistance(lflDistance);
        double lrlMod = changeFromDistance(lrlDistance);
        double rflMod = changeFromDistance(rflDistance);
        double rrlMod = changeFromDistance(rrlDistance);

        lfl.setPower((-1 * power) + lflMod);
        lrl.setPower((-1 * power) + lrlMod);
        rfl.setPower(power + rflMod);
        rrl.setPower(power + rrlMod);
    }
    public void rotate(double lDepth, double rDepth) {
        if (rDepth > 0) {
            lfd.setPower(lMod * rDepth);
            lrd.setPower(lMod * rDepth * -1.0);
            rfd.setPower(-1 * rMod * rDepth);
            rrd.setPower(-1 * rMod * rDepth);
        } else if (lDepth > 0){
            lfd.setPower(-1 * lMod * lDepth);
            lrd.setPower(-1 * lMod * lDepth * -1.0);
            rfd.setPower(rMod * lDepth);
            rrd.setPower(rMod * lDepth);
        } else {
            lfd.setPower(0.0);
            lrd.setPower(0.0);
            rfd.setPower(0.0);
            rrd.setPower(0.0);
        }
    }
    public void updateLiftDistances() {
        lflDistance = lfl.getCurrentPosition() - lflStart;
        lrlDistance = lrl.getCurrentPosition() - lrlStart;
        rflDistance = rfl.getCurrentPosition() - rflStart;
        rrlDistance = rrl.getCurrentPosition() - rrlStart;
    }
    public int averageLiftDistance() {
        return (lflDistance + lrlDistance + rflDistance + rrlDistance)/4;
    }
    public double changeFromDistance(double distance) {
        if (distance > maxPowerDistance)
            return maxChangePower;
        if (distance < (-1.0 * maxChangePower))
            return -1.0 * maxChangePower;
        if (Math.abs(distance) < goodEnoughDistance)
            return 0.0;
        return ((double) distance/maxPowerDistance) * maxChangePower;
    }
    public void launchPlane () {
        planeServo.setPosition(planeStart + planeDiff);
    }
    private void setClaw (boolean settingLeft, boolean closed) {
        if (settingLeft) {
            if (closed) {
                lClaw.setPosition(leftOpenPos + leftDiff);

            } else {
                lClaw.setPosition(leftOpenPos);
            }
        } else {
            if (closed) {
                rClaw.setPosition(rightOpenPos + rightDiff);
            } else {
                rClaw.setPosition(rightOpenPos);
            }
        }
    }
}
