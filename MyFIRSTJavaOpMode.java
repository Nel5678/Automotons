package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Template: 2024MovementOpmode1", group="Linear Opmode")
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private DcMotor lfd = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    private DcMotor lrd = hardwareMap.get(DcMotor.class, "leftRearDrive");
    private DcMotor rfd = hardwareMap.get(DcMotor.class, "rightFrontDrive");
    private DcMotor rrd = hardwareMap.get(DcMotor.class, "rightRearDrive");
    private int lMod = -1; // changes CW to CCW for left motors
    private int rMod = 1; // changes CW to CCW for right motors
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            telemetry.addData("leftX", "" + leftX);
            telemetry.addData("leftY", "" + leftY);
            if (leftX == 0.0 && leftY == 0.0) {
                // rotation stuff
            } else {
                translate(leftX, leftY);
            }
        }
    }
    private double pythag(double num1, double num2) {
        return Math.sqrt(Math.pow(num1, 2) + Math.pow(num2, 2));
    }
    // sets 4 drive motors for translational movement
    private void translate(double xVal, double yVal) {
        double totalPower = pythag(xVal, yVal);
        if (xVal == 0.0 && yVal == 0.0) {
            lfd.setPower(0.0);
            lrd.setPower(0.0);
            rfd.setPower(0.0);
            rrd.setPower(0.0);
        } else if (xVal >= 0.0 && yVal >= 0.0) {
            lrd.setPower(lMod * totalPower);
            rfd.setPower(rMod * totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(lMod * oppPower);
            rrd.setPower(rMod * oppPower);
        } else if (xVal <= 0.0 && yVal >= 0.0) {
            lfd.setPower(lMod * totalPower);
            rrd.setPower(rMod * totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(lMod * oppPower);
            rfd.setPower(rMod * oppPower);
        } else if (xVal <= 0.0 && yVal <= 0.0) {
            lrd.setPower(-1 * lMod * totalPower);
            rfd.setPower(-1 * rMod * totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(lMod * oppPower);
            rrd.setPower(rMod * oppPower);
        } else {
            lfd.setPower(-1 * lMod * totalPower);
            rrd.setPower(-1 * rMod * totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(lMod * oppPower);
            rfd.setPower(rMod * oppPower);
        }
    }

}

