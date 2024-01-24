
package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="MyAutonomousOpMode")
public class Auto extends LinearOpMode{
    private DcMotor lfd;
    private DcMotor lrd;
    private DcMotor rfd;
    private DcMotor rrd;
    @Override
    public void runOpMode () {
        telemetry.addData("Status", "running Auto");
        telemetry.update();
        waitForStart();
        int counter = 0;
        while (opModeIsActive()) {
            telemetry.addData("Status", "Auto after start");
            telemetry.addData("Counter", counter);
            counter++;
            telemetry.update();
        }
    }
}
