package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class TrajectoryBuilder {
    public AutoBase.Coordinates c;
    public SampleMecanumDrive drive;

    public TrajectoryBuilder(AutoBase.Coordinates coordinates, SampleMecanumDrive drive){
        this.c = coordinates;
        this.drive = drive;

        trajectorySequenceLeft = computeTrajectory(propLocations.LEFT, drive, c);
        trajectorySequenceCenter = computeTrajectory(propLocations.CENTER, drive, c);
        trajectorySequenceRight = computeTrajectory(propLocations.RIGHT, drive, c);
    }

    public ArrayList<TrajectorySequence> computeTrajectory(propLocations propLoc, SampleMecanumDrive drive, AutoBase.Coordinates c) {
        ArrayList<TrajectorySequence> finalTrajectory = new ArrayList<>();

        Vector2d backdropIntermediateFar = c.backdropIntermediateFar;
        Pose2d backdropIntermediateCoordinate;
        if (propLoc == propLocations.LEFT) {
            teamPropCoordinate = c.leftTeamProp;
            backdropIntermediateCoordinate = c.backdropIntermediateLeft;
            backdropCoordinate = c.backdropLeft;
            backdropStrafeCoordinate = c.backdropRight;
        }
        else if (propLoc == propLocations.RIGHT) {
            teamPropCoordinate = c.rightTeamProp;
            backdropIntermediateCoordinate = c.backdropIntermediateRight;
            backdropCoordinate = c.backdropRight;
            backdropStrafeCoordinate = c.backdropLeft;

        }
        else {
            teamPropCoordinate = c.centerTeamProp;
            backdropIntermediateCoordinate = c.backdropIntermediateCenter;
            backdropCoordinate = c.backdropCenter;
            backdropStrafeCoordinate = c.backdropStrafeForCenter;

        }

        assert drive != null;
        TrajectorySequenceBuilder purpleDropBuilder = drive.trajectorySequenceBuilder(c.startPose);
        if (c.startPose.getHeading()!=teamPropCoordinate.getHeading()) { // RIGHT case but could be other rotations in the future
            purpleDropBuilder = purpleDropBuilder.back(20);
        }
        purpleDropBuilder = purpleDropBuilder.lineToLinearHeading(teamPropCoordinate);
        TrajectorySequence purpleDrop = purpleDropBuilder.build();
        finalTrajectory.add(purpleDrop); //get 0


        TrajectorySequence setupForBackdropNear = drive.trajectorySequenceBuilder(purpleDrop.end())
                .lineTo(c.setupForBackdrop)
                .build();

        TrajectorySequence setupForBackdropFar = drive.trajectorySequenceBuilder(purpleDrop.end())
                .forward(8)
                .lineTo(c.prepareFarDrop, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(backdropIntermediateFar, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Pose2d endPurpledrop;
        if (c.isNearSide) {
            finalTrajectory.add(setupForBackdropNear);
            endPurpledrop = setupForBackdropNear.end();
        } else{
            finalTrajectory.add(setupForBackdropFar);
            endPurpledrop = setupForBackdropFar.end();
        } // get 1

        TrajectorySequence goToBackdrop;
        if (c.isNearSide) {
            goToBackdrop = drive.trajectorySequenceBuilder(endPurpledrop)
                    .lineToSplineHeading(backdropIntermediateCoordinate)
                    .lineToLinearHeading(backdropCoordinate)
                                .build();
        }
        else {
            goToBackdrop = drive.trajectorySequenceBuilder(endPurpledrop)
                    // Stop 3 inches before touching backdrop so that heading / robot pivoting is smooth and doesn't scratch backdrop
                    .lineToSplineHeading(new Pose2d(backdropStrafeCoordinate.getX() - 3,
                            backdropStrafeCoordinate.getY(),
                            backdropStrafeCoordinate.getHeading()))
                    .back(3)
                    .lineToLinearHeading(backdropCoordinate, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        }

        finalTrajectory.add(goToBackdrop); // get 2

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .lineTo(c.parkInCorner)
                .back(10)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .forward(2)
                .build();

        if (c.isNearSide) {
            finalTrajectory.add(parkRight);
        }else{
            finalTrajectory.add(parkLeft);
        }

        return finalTrajectory;
    }

    public enum propLocations{
        LEFT,
        CENTER,
        RIGHT
    }


    public ArrayList<TrajectorySequence> trajectorySequenceLeft, trajectorySequenceCenter, trajectorySequenceRight;


    Pose2d teamPropCoordinate;
    Pose2d backdropCoordinate;
    Pose2d backdropStrafeCoordinate;

}
