package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryTester {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24.2649, 61.75, 1.83825, 1.0471975511965976, 15)
                .setDimensions(14.5,16.6)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -63, 89.5301))

                                .addTemporalMarker(0.5, 0.1, () -> {
                                    // This example will run 50% of the way through the path, plus 0.1 seconds
                                    // The offset can be left at zero but is useful for making slight adjustments to the timing
                                })
                                .build()
                )
                ;



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}
