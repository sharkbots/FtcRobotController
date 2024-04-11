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

        // ACTION 0: Drop pixel on spike mark
        TrajectorySequenceBuilder purpleDropBuilder = drive.trajectorySequenceBuilder(c.startPose);
        if (c.startPose.getHeading()!=teamPropCoordinate.getHeading()) { // RIGHT case but could be other rotations in the future
            purpleDropBuilder = purpleDropBuilder.back(20);
        }
        purpleDropBuilder = purpleDropBuilder.lineToLinearHeading(teamPropCoordinate);
        TrajectorySequence purpleDrop = purpleDropBuilder.build();
        finalTrajectory.add(purpleDrop); //get 0


        // ACTION 1: setup for backdrop / setup to get pixel from stack
        // near side
        TrajectorySequenceBuilder setupForBackdropNearBuilder = drive.trajectorySequenceBuilder(purpleDrop.end())
                .lineTo(c.setupForBackdrop);
        TrajectorySequence setupForBackdropNear = setupForBackdropNearBuilder.build();

        // far side
        TrajectorySequenceBuilder purpleDropToStackSetupBuilder = drive.trajectorySequenceBuilder(purpleDrop.end());
        if (propLoc == propLocations.RIGHT) {
            purpleDropToStackSetupBuilder = purpleDropToStackSetupBuilder.lineTo(c.purpleDropToStackPreSetup);
        }
        // lineToLinearHeading might not be possible here for right pos, lineTo + turn
        purpleDropToStackSetupBuilder = purpleDropToStackSetupBuilder.lineToLinearHeading(c.purpleDropToStackSetup);
        TrajectorySequence purpleDropToStackSetup = purpleDropToStackSetupBuilder.build();

        Pose2d endPurpleDrop;
        if (c.isNearSide) {
            finalTrajectory.add(setupForBackdropNear);
            endPurpleDrop = setupForBackdropNear.end();
        } else{
            finalTrajectory.add(purpleDropToStackSetup);
            endPurpleDrop = purpleDropToStackSetup.end();
        } // get 1


        // ACTION 2: go to backdrop / go to stacks (from Purple Drop)
        // near side
        TrajectorySequenceBuilder goToBackdropBuilder = drive.trajectorySequenceBuilder(endPurpleDrop)
                .lineToSplineHeading(backdropIntermediateCoordinate)
                .lineToLinearHeading(backdropCoordinate);

        if (c.startPose.getHeading()!=teamPropCoordinate.getHeading()) {
            if (c.isBlueAlliance) {
                goToBackdropBuilder = goToBackdropBuilder.strafeLeft(4);
            } else {
                goToBackdropBuilder = goToBackdropBuilder.strafeRight(6);
            }
        }
        TrajectorySequence goToBackdrop = goToBackdropBuilder.build();

        // far side
        TrajectorySequenceBuilder goToStackPDSetupBuilder = drive.trajectorySequenceBuilder(endPurpleDrop)
                .lineToLinearHeading(c.stackLeftSetup);
        TrajectorySequence goToStackPDSetup = goToStackPDSetupBuilder.build();

        TrajectorySequenceBuilder goToStackBuilder = drive.trajectorySequenceBuilder(endPurpleDrop)
                .lineToLinearHeading(c.stackLeft);
        TrajectorySequence goToStack = goToStackBuilder.build();

        if (c.isNearSide) {
            finalTrajectory.add(goToBackdrop); // get 1
        } else{
            finalTrajectory.add(goToStackPDSetup); // get 1
            finalTrajectory.add(goToStack); // get
        }




        TrajectorySequence setupForBackdropFar = drive.trajectorySequenceBuilder(purpleDrop.end())
                .forward(8)
                .lineTo(c.prepareFarDrop, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(backdropIntermediateFar, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /*
        TrajectorySequenceBuilder goToBackdropBuilder = drive.trajectorySequenceBuilder(endPurpleDrop);
        if (c.isNearSide) {
            goToBackdropBuilder = goToBackdropBuilder
                    .lineToSplineHeading(backdropIntermediateCoordinate)
                    .lineToLinearHeading(backdropCoordinate);

            if (c.startPose.getHeading()!=teamPropCoordinate.getHeading()){
                if (c.isBlueAlliance) {
                    goToBackdropBuilder = goToBackdropBuilder.strafeLeft(4);
                } else {
                    goToBackdropBuilder = goToBackdropBuilder.strafeRight(6);
                }
            }
        }
        else {
            if (propLoc != propLocations.CENTER) {
                goToBackdropBuilder = goToBackdropBuilder
                    // Stop 3 inches before touching backdrop so that heading / robot pivoting is smooth and doesn't scratch backdrop
                    .lineToSplineHeading(new Pose2d(backdropStrafeCoordinate.getX() - 3,
                            backdropStrafeCoordinate.getY(),
                            backdropStrafeCoordinate.getHeading()))
                    .back(3)
                    .lineToLinearHeading(backdropCoordinate, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL));
            }
            else {
                goToBackdropBuilder = goToBackdropBuilder
                        .lineToSplineHeading(backdropIntermediateCoordinate)
                        .lineToLinearHeading(backdropCoordinate);
                if (c.isBlueAlliance) {
                    goToBackdropBuilder = goToBackdropBuilder.strafeRight(1);
                } else {
                    goToBackdropBuilder = goToBackdropBuilder.strafeLeft(3);
                }
            }
        }
        TrajectorySequence goToBackdrop = goToBackdropBuilder.build();
        finalTrajectory.add(goToBackdrop); // get 2*/

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
