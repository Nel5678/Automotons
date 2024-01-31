
package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="MyAutonomousOpMode")
public class Auto extends LinearOpMode {
    // motors
    private DcMotor lfd;
    private DcMotor lrd;
    private DcMotor rfd;
    private DcMotor rrd;
    // motor directions
    private DcMotorSimple.Direction forward = DcMotorSimple.Direction.FORWARD;
    private DcMotorSimple.Direction reverse = DcMotorSimple.Direction.REVERSE;
    // claws
    private Servo lClaw;
    private Servo rClaw;
    // movement variables
    private int desiredDistance = 6000;
    private double defaultPower = 0.5;
    private double maxChangePower = 0.2;
    private int goodEnoughDistance = 30;
    private int maxPowerDistance = 500;
    private int lfdStart;
    private int lrdStart;
    private int rfdStart;
    private int rrdStart;
    private int lfdDistance;
    private int lrdDistance;
    private int rfdDistance;
    private int rrdDistance;
    // whether the pixel has been dropped yet
    private boolean dropPixel = false;
    @Override
    public void runOpMode () {
        lfd = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        lrd = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rfd = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rrd = hardwareMap.get(DcMotor.class, "rightRearDrive");

        lfd.setDirection(forward);
        lrd.setDirection(reverse);
        rfd.setDirection(reverse);
        rrd.setDirection(forward);

        lfdStart = lfd.getCurrentPosition();
        lrdStart = lrd.getCurrentPosition();
        rfdStart = rfd.getCurrentPosition();
        rrdStart = rrd.getCurrentPosition();

        updateDriveDistances();

        rClaw = hardwareMap.get(Servo.class, "rightClaw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !dropPixel) {
            MyFIRSTJavaOpMode.setClaw(false, true, rClaw);

            updateDriveDistances();

            double lfdMod = changeFromDistance(lfdDistance);
            double lrdMod = changeFromDistance(lrdDistance);
            double rfdMod = changeFromDistance(rfdDistance);
            double rrdMod = changeFromDistance(rrdDistance);

            lfd.setPower(defaultPower + lfdMod);
            lrd.setPower(defaultPower + lrdMod);
            rfd.setPower(defaultPower + rfdMod);
            rrd.setPower(defaultPower + rrdMod);

            if (averageDriveDistance() >= desiredDistance)
                dropPixel = true;

            telemetry.addData("avgDistance", averageDriveDistance());
            telemetry.update();
        }
        while (opModeIsActive()) {
            MyFIRSTJavaOpMode.setClaw(false, false, rClaw);
            telemetry.addData("avgDistance", averageDriveDistance());
            telemetry.update();
        }
    }
    public void translate(double xVal, double yVal) {
        MyFIRSTJavaOpMode.translate(xVal, yVal, lfd, lrd, rfd, rrd);
    }
    public void updateDriveDistances() {
        lfdDistance = lfd.getCurrentPosition() - lfdStart;
        lrdDistance = lrd.getCurrentPosition() - lrdStart;
        rfdDistance = rfd.getCurrentPosition() - rfdStart;
        rrdDistance = rrd.getCurrentPosition() - rrdStart;
    }
    public int averageDriveDistance () {
        return (lfdDistance + lrdDistance + rfdDistance + rrdDistance)/4;
    }
    public double changeFromDistance(double distance) {
        distance -= averageDriveDistance();
        if (distance > maxPowerDistance)
            return maxChangePower;
        if (distance < (-1.0 * maxChangePower))
            return -1.0 * maxChangePower;
        if (Math.abs(distance) < goodEnoughDistance)
            return 0.0;
        return ((double) distance/maxPowerDistance) * maxChangePower;
    }
}
