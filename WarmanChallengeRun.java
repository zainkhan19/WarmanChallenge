package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Config
@TeleOp(group = "drive")
public class WarmanChallengeRun extends LinearOpMode {
    public Servo leftServo;
    public Servo rightServo;
    ColorSensor color;
    int trigger = 0;


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        color = hardwareMap.get(ColorSensor.class, "Colour");
        leftServo = hardwareMap.get(Servo.class, "LeftServo");
        rightServo = hardwareMap.get(Servo.class, "RightServo");


        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                // todo - initialize start (straight speed to 25% and turn speeds to 50%)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(12,DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(12))
                .setTurnConstraint(Math.toRadians(70),Math.toRadians(70))
                .forward(3.5) // initialize start

                // todo - intake first ball

                // lower intake arm to 0 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0))
                .addTemporalMarker(() -> rightServo.setPosition(1))
                .waitSeconds(2)

                // raise intake arm to 30 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.3))
                .addTemporalMarker(() -> rightServo.setPosition(0.8))
                .waitSeconds(0.2)

                // raise intake arm to 50 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.4))
                .addTemporalMarker(() -> rightServo.setPosition(0.7))
                .waitSeconds(0.5)

                // raise intake arm to 90 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.6))
                .addTemporalMarker(() -> rightServo.setPosition(0.5))
                .waitSeconds(0.5)

                // todo - transition to second ball

                .turn(Math.toRadians(-7)) // turn to straighten again wall

                .strafeRight(14.8) // strafe to second ball

                .waitSeconds(0.2) // wait for stability

                // todo - intake second ball

                .forward(3.5)  // move forward to intake second ball


                // lower intake arm to 50 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.4))
                .addTemporalMarker(() -> rightServo.setPosition(0.7))
                .waitSeconds(0.8)

                // raise intake arm to 70 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.42))
                .addTemporalMarker(() -> rightServo.setPosition(0.68))
                .waitSeconds(0.3)

                // raise intake arm to 80 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.5))
                .addTemporalMarker(() -> rightServo.setPosition(0.6))
                .waitSeconds(0.2)

                // raise intake arm to 90 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.6))
                .addTemporalMarker(() -> rightServo.setPosition(0.5))
                .waitSeconds(0.2)


                // todo - transition to third ball

                .strafeRight(13) // strafe to second ball


                .turn(Math.toRadians(7)) // correction turn

                .waitSeconds(0.5)


                // todo - intake third ball

                .forward(0.7) // move forward to intake ball

                // lower intake arm to 0 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0))
                .addTemporalMarker(() -> rightServo.setPosition(1))
                .waitSeconds(1.4)

                // raise intake arm to 30 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.3))
                .addTemporalMarker(() -> rightServo.setPosition(0.8))
                .waitSeconds(0.2)

                // raise intake arm to 50 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.4))
                .addTemporalMarker(() -> rightServo.setPosition(0.7))
                .waitSeconds(0.5)

                // raise intake arm to 90 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.6))
                .addTemporalMarker(() -> rightServo.setPosition(0.5))
                .waitSeconds(0.5)

                // todo - transition to fourth ball

                .back(7)
                .turn(Math.toRadians(181)) // correction turn
                .waitSeconds(0.5)
                .forward(20)
                .turn(Math.toRadians(8)) // correction turn
                .waitSeconds(0.2)

                // todo - intake fourth ball

                // lower intake arm to 0 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0))
                .addTemporalMarker(() -> rightServo.setPosition(1))
                .waitSeconds(1.7)

                // raise intake arm to 30 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.3))
                .addTemporalMarker(() -> rightServo.setPosition(0.8))
                .waitSeconds(0.5)

                // raise intake arm to 50 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.4))
                .addTemporalMarker(() -> rightServo.setPosition(0.7))
                .waitSeconds(0.5)

                // todo - transition to fifth ball

                .back(2.5)
                .turn(Math.toRadians(-7.5))
                .strafeRight(12.5) // strafe to fifth ball


                // todo - intake fifth ball

                // raise intake arm to 30 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.3))
                .addTemporalMarker(() -> rightServo.setPosition(0.8))
                .waitSeconds(1)


                // raise intake arm to 50 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.375))
                .addTemporalMarker(() -> rightServo.setPosition(0.725))
                .waitSeconds(0.5)

                // raise intake arm to 50 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.4))
                .addTemporalMarker(() -> rightServo.setPosition(0.7))
                .waitSeconds(0.5)

                // raise intake arm to 90 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.6))
                .addTemporalMarker(() -> rightServo.setPosition(0.5))
                .waitSeconds(0.5)

                // todo - transition to sixth ball

                .strafeRight(13.45) // strafe to fifth ball
                .forward(1.1) // move forward to intake ball

                // todo - Intake sixth ball

                // lower intake arm to 0 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0))
                .addTemporalMarker(() -> rightServo.setPosition(1))
                .waitSeconds(1.7)

                // raise intake arm to 30 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.3))
                .addTemporalMarker(() -> rightServo.setPosition(0.8))
                .waitSeconds(0.2)

                // raise intake arm to 50 degrees
                .addTemporalMarker(() -> leftServo.setPosition(0.4))
                .addTemporalMarker(() -> rightServo.setPosition(0.7))
                .waitSeconds(0.5)

                // todo - move to incinerator hole

                .resetConstraints()
                .resetTurnConstraint()
                .back(4)
                .strafeLeft(1.3)
                .turn(Math.toRadians(70))
                .turn(Math.toRadians(-65))
                .forward(3)
                .back(3)

                .build();


        waitForStart();

        if (!isStopRequested())
            while (getRuntime() < 10000) {
                if(color.red() > 1000 && color.green() > 1000 && color.blue() > 1000) {
                    trigger = 1;
                }
                if( trigger == 1) {
                    drive.followTrajectorySequence(trajSeq);
                    trigger = 0;
                }
            }
    }
}