
package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="MyAutonomousOpMode")
public class MyAutonomousOpMode extends Autonomous{
    private DcMotor lfd;
    private DcMotor lrd;
    private DcMotor rfd;
    private DcMotor rrd;
    @Override public void main() throws InterruptedException
    {
        lfd = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        lrd = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rfd = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rrd = hardwareMap.get(DcMotor.class, "rightRearDrive");
        lrd.setTargetPosition(lrd.getCurrentPosition() + 1000);
        while (lrd.getCurrentPosition()) {

        }
    }
}
