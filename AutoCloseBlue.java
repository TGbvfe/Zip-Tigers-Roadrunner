package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "CloseBlue", group = "examples")
public class AutoCloseBlue extends LinearOpMode {

    public Servo Grasp;
    //public Servo armAngle;
    public Servo plane;
    public DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        armMotor = hardwareMap.dcMotor.get("armMotor");
        Grasp = hardwareMap.get(Servo.class,"Grasp");
        //armAngle = hardwareMap.get(Servo.class, "armAngle");
        plane = hardwareMap.get(Servo.class, "plane");

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(10.98, 67.17, Math.toRadians(-88.36)))
                .splineTo(new Vector2d(43.04, 48.18), Math.toRadians(5.71))
                .splineTo(new Vector2d(53.32, 59.86), Math.toRadians(180.00))

                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(traj.start());
        drive.followTrajectorySequence(traj);
    }
}
