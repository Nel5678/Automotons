package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Template: 2024 Brie 3", group="Linear Opmode")
public class MyFIRSTJavaOpMode extends LinearOpMode {
    // motor directions
    private DcMotorSimple.Direction forward = DcMotor.Direction.FORWARD;
    private DcMotorSimple.Direction reverse = DcMotor.Direction.REVERSE;
    // drive stuff
    private DcMotor lfd;
    private DcMotor lrd;
    private DcMotor rfd;
    private DcMotor rrd;
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
    private int liftTarget = 0;
    // plane stuff
    private Servo planeServo;
    private double planeStart = 0.5;
    private double planeDiff = 0.5;
    // claw stuff
    private Servo lClaw;
    private Servo rClaw;
    private static double leftOpenPos = 0.703125;
    private static double leftDiff = (double) -3/8;
    private static double rightOpenPos = 0.296875;
    private static double rightDiff = (double) 3/8;
    // drum stuff

    @Override
    public void runOpMode() {
        lfd = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        lrd = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rfd = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rrd = hardwareMap.get(DcMotor.class, "rightRearDrive");

        lfd.setDirection(forward);
        lrd.setDirection(forward);
        rfd.setDirection(reverse);
        rrd.setDirection(reverse);

        lfl = hardwareMap.get(DcMotor.class, "leftFrontLift");
        lrl = hardwareMap.get(DcMotor.class, "leftRearLift");
        rfl = hardwareMap.get(DcMotor.class, "rightFrontLift");
        rrl = hardwareMap.get(DcMotor.class, "rightRearLift");

        lfl.setDirection(reverse);
        lrl.setDirection(forward);
        rfl.setDirection(reverse);
        rrl.setDirection(reverse);

        lflStart = lfl.getCurrentPosition();
        lrlStart = lrl.getCurrentPosition();
        rflStart = rfl.getCurrentPosition();
        rrlStart = rrl.getCurrentPosition();

        lflDistance = 0;
        lrlDistance = 0;
        rflDistance = 0;
        rrlDistance = 0;
/*
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        planeServo.setPosition(planeStart);

        lClaw = hardwareMap.get(Servo.class, "leftClaw");
        rClaw = hardwareMap.get(Servo.class, "rightClaw");
*/

        boolean leftPressed = false;
        boolean rightPressed = false;
        boolean leftClosed = false;
        boolean rightClosed = false;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("leftRearPosition", lrd.getCurrentPosition());
            double leftX = gamepad1.left_stick_x * 0.1;
            double leftY = gamepad1.left_stick_y * -0.1;
            double rightY = gamepad1.right_stick_y * -0.3;
            telemetry.addData("leftX", "" + leftX);
            telemetry.addData("leftY", "" + leftY);
            telemetry.addData("rightY", "" + rightY);
            /*if (leftX == 0.0 && leftY == 0.0) {
                double rDepth = gamepad1.right_trigger;
                double lDepth = gamepad1.left_trigger;
                rotate(lDepth, rDepth);
                telemetry.addData("", "rotate called");
            } else {
                translate(leftX, leftY);
                telemetry.addData("", "translateCalled");
            }*/
            //addDriveTelemetry();
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
            lift(rightY);
            addLiftTelemetry();
            telemetry.update();
        }

    }
    public void addDriveTelemetry () {
        telemetry.addData("lfd power", lfd.getPower() + ", pos: " + lfd.getCurrentPosition());
        telemetry.addData("lrd power", lrd.getPower() + ", pos: " + lrd.getCurrentPosition());
        telemetry.addData("rfd power", lfd.getPower() + ", pos: " + rfd.getCurrentPosition());
        telemetry.addData("rrd power", rrd.getPower() + ", pos: " + rrd.getCurrentPosition());
    }
    public void addLiftTelemetry () {
        telemetry.addData("lfl power", lfl.getPower() + ", dist: " + lflDistance);
        telemetry.addData("lrl power", lrl.getPower() + ", dist: " + lrlDistance);
        telemetry.addData("rfl power", rfl.getPower() + ", dist: " + rflDistance);
        telemetry.addData("rrl power", rrl.getPower() + ", dist: " + rrlDistance);
    }
    public static double pythag(double num1, double num2) {
        return Math.sqrt(Math.pow(num1, 2) + Math.pow(num2, 2));
    }
    public static void translate(double xVal, double yVal, DcMotor lfd, DcMotor lrd, DcMotor rfd, DcMotor rrd) {
        double totalPower = pythag(xVal, yVal);
        if (totalPower > 1) totalPower = 1;
        if (totalPower < -1) totalPower = -1;
        if (xVal == 0.0 && yVal == 0.0) {
            lfd.setPower(0.0);
            lrd.setPower(0.0);
            rfd.setPower(0.0);
            rrd.setPower(0.0);
        } else if (xVal >= 0.0 && yVal >= 0.0) {
            lrd.setPower(totalPower);
            rfd.setPower(totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(oppPower);
            rrd.setPower(oppPower);
        } else if (xVal <= 0.0 && yVal >= 0.0) {
            lfd.setPower(totalPower);
            rrd.setPower(totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(oppPower);
            rfd.setPower(oppPower);
        } else if (xVal <= 0.0 && yVal <= 0.0) {
            lrd.setPower(-1 * totalPower);
            rfd.setPower(-1 * totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(oppPower);
            rrd.setPower(oppPower);
        } else {
            lfd.setPower(-1 * totalPower);
            rrd.setPower(-1 * totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(oppPower);
            rfd.setPower(oppPower);
        }
    }
    public void translate(double xVal, double yVal) {
        translate(xVal, yVal, lfd, lrd, rfd, rrd);
    }
    public void rotate(double lDepth, double rDepth) {
        if (rDepth > 0) {
            lfd.setPower(rDepth);
            lrd.setPower(rDepth);
            rfd.setPower(-1 * rDepth);
            rrd.setPower(-1 * rDepth);
        } else if (lDepth > 0){
            lfd.setPower(-1 * lDepth);
            lrd.setPower(-1 * lDepth);
            rfd.setPower(lDepth);
            rrd.setPower(lDepth);
        } else {
            lfd.setPower(0.0);
            lrd.setPower(0.0);
            rfd.setPower(0.0);
            rrd.setPower(0.0);
        }
    }
    public void lift(double power) {
        updateLiftDistances();
        double lflMod = changeFromDistance(lflDistance);
        double lrlMod = changeFromDistance(lrlDistance);
        double rflMod = changeFromDistance(rflDistance);
        double rrlMod = changeFromDistance(rrlDistance);

        lfl.setPower(power + lflMod);
        lrl.setPower(power + lrlMod);
        rfl.setPower(power + rflMod);
        rrl.setPower(power + rrlMod);
    }
    public void updateLiftDistances() {
        lflDistance = (lfl.getCurrentPosition() - lflStart) * -1;
        lrlDistance = lrl.getCurrentPosition() - lrlStart;
        rflDistance = rfl.getCurrentPosition() - rflStart;
        rrlDistance = rrl.getCurrentPosition() - rrlStart;
    }
    public int averageLiftDistance() {
        return (lflDistance + lrlDistance + rflDistance + rrlDistance)/4;
    }
    public double changeFromDistance(double distance) {
        distance -= averageLiftDistance();
        if (distance > maxPowerDistance)
            return maxChangePower;
        if (distance < (-1.0 * maxPowerDistance))
            return -1.0 * maxChangePower;
        if (Math.abs(distance) < goodEnoughDistance)
            return 0.0;
        return ((double) distance/maxPowerDistance) * maxChangePower;
    }
    public void launchPlane () {
        planeServo.setPosition(planeStart + planeDiff);
    }
    public static void setClaw (boolean settingLeft, boolean closed, Servo claw) {
        if (settingLeft) {
            if (closed) {
                claw.setPosition(leftOpenPos + leftDiff);
            } else {
                claw.setPosition(leftOpenPos);
            }
        } else {
            if (closed) {
                claw.setPosition(rightOpenPos + rightDiff);
            } else {
                claw.setPosition(rightOpenPos);
            }
        }
    }
    public void setClaw (boolean settingLeft, boolean closed) {
        if (settingLeft)
            setClaw(true, closed, lClaw);
        else
            setClaw(false, closed, rClaw);
    }
}

