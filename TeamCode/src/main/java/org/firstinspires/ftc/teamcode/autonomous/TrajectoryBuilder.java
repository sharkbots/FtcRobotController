package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class TrajectoryBuilder {
    AutoBase.ConfigItems config;
    private final AutoBase.Coordinates c;
    private final SampleMecanumDrive drive;

    private PROP_LOCATIONS propLocation;

    public enum PROP_LOCATIONS {
        LEFT,
        CENTER,
        RIGHT
    }


    public TrajectoryBuilder(AutoBase.Coordinates coordinates, SampleMecanumDrive drive, AutoBase.ConfigItems config){
        this.config = config;
        this.c = coordinates;
        this.drive = drive;

        propLocation = PROP_LOCATIONS.LEFT;
        trajectorySequenceLeft = computeTrajectory();

        propLocation = PROP_LOCATIONS.CENTER;
        trajectorySequenceCenter = computeTrajectory();

        propLocation = PROP_LOCATIONS.RIGHT;
        trajectorySequenceRight = computeTrajectory();
    }

    public ArrayList<TrajectorySequence> computeTrajectory() {
        ArrayList<TrajectorySequence> finalTrajectory = new ArrayList<>();

/*

        Vector2d backdropIntermediateFar = c.backdropIntermediateFarOutside;
        Pose2d stackCoordinate;
        Pose2d stackSetupCoordinate;
        Vector2d trussCoordinate;

        if (stackLocation == STACK_LOCATIONS.LEFT) {
            stackSetupCoordinate = c.leftStackSetup;
            stackCoordinate = c.leftStack;
        } else if (stackLocation == STACK_LOCATIONS.CENTER) {
            stackSetupCoordinate = c.centerStackSetup;
            stackCoordinate = c.centerStack;
        } else {
            stackSetupCoordinate = c.rightStackSetup;
            stackCoordinate = c.rightStack;
        }

        // ACTION 0: Drop pixel on spike mark
        TrajectorySequenceBuilder purpleDropBuilder = drive.trajectorySequenceBuilder(c.startPose);
        if (c.startPose.getHeading()!=teamPropCoordinate.getHeading()) { // RIGHT case but could be other rotations in the future
            purpleDropBuilder = purpleDropBuilder.back(20);
        }
        purpleDropBuilder = purpleDropBuilder.lineToLinearHeading(teamPropCoordinate);
        TrajectorySequence purpleDrop = purpleDropBuilder.build();
        finalTrajectory.add(purpleDrop); //get 0



        if (propLocation == PROP_LOCATIONS.LEFT) {


//            teamPropCoordinate = c.leftTeamProp;
//            backdropIntermediateCoordinate = c.backdropIntermediateLeft;
//            backdropCoordinate = c.backdropLeft;
//            backdropStrafeCoordinate = c.backdropRight;
        }
        else if (propLocation == PROP_LOCATIONS.RIGHT) {

//            teamPropCoordinate = c.rightTeamProp;
//            backdropIntermediateCoordinate = c.backdropIntermediateRight;
//            backdropCoordinate = c.backdropRight;
//            backdropStrafeCoordinate = c.backdropLeft;

        }
        else {

//            teamPropCoordinate = c.centerTeamProp;
//            backdropIntermediateCoordinate = c.backdropIntermediateCenter;
//            backdropCoordinate = c.backdropCenter;
//            backdropStrafeCoordinate = c.backdropStrafeForCenter;

        }

        assert drive != null;






        // ACTION 1: setup for backdrop / setup to get pixel from stack
        // near side
        TrajectorySequenceBuilder setupForBackdropNearBuilder = drive.trajectorySequenceBuilder(purpleDrop.end())
                .lineTo(c.setupForBackdrop);
        TrajectorySequence setupForBackdropNear = setupForBackdropNearBuilder.build();

        // far side
        TrajectorySequenceBuilder purpleDropToStackSetupBuilder = drive.trajectorySequenceBuilder(purpleDrop.end());
        if (propLocation == PROP_LOCATIONS.RIGHT) {
            purpleDropToStackSetupBuilder = purpleDropToStackSetupBuilder.lineTo(c.purpleDropToStackPreSetup);
        }
        // lineToLinearHeading might not be possible here for right pos, lineTo + turn
        purpleDropToStackSetupBuilder = purpleDropToStackSetupBuilder.lineToLinearHeading(c.purpleDropToStackSetup);
        TrajectorySequence purpleDropToStackSetup = purpleDropToStackSetupBuilder.build();

        Pose2d endPurpleDrop;
        if (config.side == AutoBase.SIDE.NEAR) {
            finalTrajectory.add(setupForBackdropNear);
            endPurpleDrop = setupForBackdropNear.end();
        } else{
            finalTrajectory.add(purpleDropToStackSetup);
            endPurpleDrop = purpleDropToStackSetup.end();
        } // get 1


        // ACTION 2: go to backdrop / go to stacks (from Purple Drop)
        // near side
        TrajectorySequenceBuilder goToBackdropBuilder = drive.trajectorySequenceBuilder(endPurpleDrop)
                //.lineToSplineHeading(backdropIntermediateCoordinate)
                .lineToLinearHeading(backdropCoordinate);

        if (c.startPose.getHeading()!=teamPropCoordinate.getHeading()) {
            if (config.alliance == AutoBase.ALLIANCE.BLUE) {
                goToBackdropBuilder = goToBackdropBuilder.strafeLeft(4);
            } else {
                goToBackdropBuilder = goToBackdropBuilder.strafeRight(6);
            }
        }
        TrajectorySequence goToBackdrop = goToBackdropBuilder.build();

        // far side
        TrajectorySequenceBuilder goToStackPDSetupBuilder = drive.trajectorySequenceBuilder(endPurpleDrop)
                .lineToLinearHeading(c.leftStackSetup); // stack location
        TrajectorySequence goToStackPDSetup = goToStackPDSetupBuilder.build();

        TrajectorySequenceBuilder goToStackBuilder = drive.trajectorySequenceBuilder(endPurpleDrop)
                .lineToLinearHeading(c.leftStack);
        TrajectorySequence goToStack = goToStackBuilder.build();

        if (config.side == AutoBase.SIDE.NEAR) {
            finalTrajectory.add(goToBackdrop); // get 1
        } else{
            finalTrajectory.add(goToStackPDSetup); // get 1
            finalTrajectory.add(goToStack); // get 2
        }




        TrajectorySequence setupForBackdropFar = drive.trajectorySequenceBuilder(purpleDrop.end())
                .forward(8)
                .lineTo(c.prepareFarDropOutside, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(backdropIntermediateFar, SampleMecanumDrive.getVelocityConstraint(15, 15, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // CYCLE CODE
        // TODO: include temporal or displacement markers, create var for after which trajectory cycle starts
        TrajectorySequenceBuilder stackToBackdropBuilder = drive.trajectorySequenceBuilder(goToStackPDSetup.end())
                .lineToLinearHeading(stackSetupCoordinate);
        if (trussLocation == TRUSS_LOCATIONS.STAGE_DOOR) {
            stackToBackdropBuilder = stackToBackdropBuilder.lineTo(c.prepareFarDropOutside);
        }
        stackToBackdropBuilder = stackToBackdropBuilder
                .lineTo(trussCoordinate)
                .lineToLinearHeading(backdropCoordinate);
        TrajectorySequence stackToBackdrop = stackToBackdropBuilder.build();

        TrajectorySequenceBuilder backdropToStackBuilder = drive.trajectorySequenceBuilder(goToStackPDSetup.end())
                .lineTo(trussCoordinate);
        if (trussLocation == TRUSS_LOCATIONS.STAGE_DOOR) {
            stackToBackdropBuilder = stackToBackdropBuilder.lineTo(c.prepareFarDropOutside);
        }
        stackToBackdropBuilder = stackToBackdropBuilder
                .lineToLinearHeading(stackSetupCoordinate);
        TrajectorySequence backdropToStack = backdropToStackBuilder.build();

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
        finalTrajectory.add(goToBackdrop); // get 2

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .lineTo(c.parkInCorner)
                .back(10)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .forward(2)
                .build();

        if (config.side == AutoBase.SIDE.NEAR) {
            finalTrajectory.add(parkRight);
        }else{
            finalTrajectory.add(parkLeft);
        }
*/
        return finalTrajectory;
    }


    public ArrayList<TrajectorySequence> trajectorySequenceLeft, trajectorySequenceCenter, trajectorySequenceRight;


    Pose2d teamPropCoordinate;
    Pose2d backdropCoordinate;
    Pose2d backdropStrafeCoordinate;

}
