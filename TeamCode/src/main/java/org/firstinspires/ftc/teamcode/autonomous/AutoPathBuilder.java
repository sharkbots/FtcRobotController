package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class AutoPathBuilder {
    private final AutoBase.Coordinates c;
    private final SampleMecanumDrive drive;

    private PROP_LOCATIONS propLocation;
    private STACK_LOCATIONS stackLocation;
    private TRUSS_LOCATIONS trussLocation;


    public enum PROP_LOCATIONS {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum STACK_LOCATIONS {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum TRUSS_LOCATIONS {
        OUTSIDE,
        CENTER,
        STAGE_DOOR
    }



    public AutoPathBuilder(AutoBase.Coordinates coordinates, SampleMecanumDrive drive){
        this.c = coordinates;
        this.drive = drive;

        propLocation = PROP_LOCATIONS.LEFT;
        trajectorySequenceLeft = computeTrajectory();

        propLocation = PROP_LOCATIONS.CENTER;
        trajectorySequenceCenter = computeTrajectory();

        propLocation = PROP_LOCATIONS.RIGHT;
        trajectorySequenceRight = computeTrajectory();
    }

    public void setAutoConfig(STACK_LOCATIONS stackLocation, TRUSS_LOCATIONS trussLocation){
        this.stackLocation = stackLocation;
        this.trussLocation = trussLocation;

    }

    public ArrayList<TrajectorySequence> computeTrajectory() {
        ArrayList<TrajectorySequence> finalTrajectory = new ArrayList<>();


        Vector2d backdropIntermediateFar = c.backdropIntermediateFarOutside;
        Pose2d backdropIntermediateCoordinate;
        if (propLocation == PROP_LOCATIONS.LEFT) {
            teamPropCoordinate = c.leftTeamProp;
            backdropIntermediateCoordinate = c.backdropIntermediateLeft;
            backdropCoordinate = c.backdropLeft;
            backdropStrafeCoordinate = c.backdropRight;
        }
        else if (propLocation == PROP_LOCATIONS.RIGHT) {
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
                .lineTo(c.prepareFarDropOutside, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
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
        TrajectorySequenceBuilder goToBackdropBuilder = drive.trajectorySequenceBuilder(endPurpledrop);
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
            if (propLocation != PROP_LOCATIONS.CENTER) {
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
        goToBackdrop = goToBackdropBuilder.build();
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



    public ArrayList<TrajectorySequence> trajectorySequenceLeft, trajectorySequenceCenter, trajectorySequenceRight;


    Pose2d teamPropCoordinate;
    Pose2d backdropCoordinate;
    Pose2d backdropStrafeCoordinate;

}
