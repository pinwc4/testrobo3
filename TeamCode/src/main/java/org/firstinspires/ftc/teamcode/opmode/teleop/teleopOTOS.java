package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;


@TeleOp(name="OTOS")
public class teleopOTOS extends OpMode {

    SparkFunOTOS myOtos;
    Follower follower;
    SparkFunOTOS.Pose2D myOTOSpos;

    @Override
    public void init() {
        //Setup of Sparkfun OTOS
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        //Units to work in
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);
        //Specify where the sensor is mounted and if it is rotated
        myOtos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        //Push the robot to tune these values for more accuracy
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        //Takes about a half second of being still to calibrate gyro
        myOtos.calibrateImu();
        myOtos.resetTracking();
        //Tell the sensor where it is at on the field
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        //create the pedro path follower
        follower = new Follower(hardwareMap);
        follower.setPose(new Pose(0,0, 0));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        myOTOSpos = myOtos.getPosition();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
        telemetry.addData("X coordinate", String.format("%.5g",myOTOSpos.x));
        telemetry.addData("Y coordinate", String.format("%.5g",myOTOSpos.y));
        telemetry.addData("Heading angle", String.format("%.5g",Math.toDegrees(myOTOSpos.h)));
    }


}
