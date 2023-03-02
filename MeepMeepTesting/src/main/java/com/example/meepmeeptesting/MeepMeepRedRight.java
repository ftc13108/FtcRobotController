package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedRight {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(750);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24.2649, 61.75, 1.83825, 1.0471975511965976, 15)
                .setDimensions(16.7,14.4)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32, -63, -86.4))
                                //start of small pole
                                .waitSeconds(.5)                           //grip cone
                                .waitSeconds(.5)                           //pick up slide
                                .lineTo(new Vector2d(23.5,-56.5))    //going to small pole
                                .waitSeconds(1)                            //drop cone
                                //end small pole

                                //start going to cone stack
                                .lineTo(new Vector2d(23.5,-57.5))    //back up
                                .lineTo(new Vector2d(11.75,-57.5))   //strafe to the side in order to go forward
                                .lineTo(new Vector2d(11.75,-12))     //going forward preparing to turn
                                .turn(Math.toRadians(-90))                 //turn 90 degrees in order to go to cones
                                .lineTo(new Vector2d(58,-12))        // move to cone stack
                                .waitSeconds(.5)                           //pick up slide
                                .waitSeconds(.5)                           //pick up cone
                                //end going to cone stack

                                //start of medium pole
                                .lineTo(new Vector2d(11.25,-12))     //going backwards in order to strafe into the medium pole
                                .lineTo(new Vector2d(11.25,-23.5))   //strafing into the medium pole area
                                .lineTo(new Vector2d(13.5,-23.5))    //going up to pole to drop the cone
                                .waitSeconds(1)                            //drop cone
                                //end medium pole

                                //going back to cone stack
                                .lineTo(new Vector2d(11.25,-23.5))   //strafing into the medium pole area
                                .lineTo(new Vector2d(11.75,-12))     //going to the side to prepare to go to the cone stack
                                .lineTo(new Vector2d(58,-12))        // move to cone stack
                                .waitSeconds(.5)                           //pick up slide
                                .waitSeconds(.5)                           //pick up cone
                                //end going to cone stack

                                //start of big pole
                                .lineTo(new Vector2d(11.25,-12))    //going backwards in order to strafe into the big pole
                                .lineTo(new Vector2d(11.25,0))      //strafing into the medium pole area
                                .lineTo(new Vector2d(13.5,0))       //going up to pole to drop the cone
                                .waitSeconds(1)                           //drop cone
                                //end big pole

                                //going to park
                                .lineTo(new Vector2d(11.25,0))      //going backwards before strafing
                                .lineTo(new Vector2d(11.75,-12))    //going to side before parking
                                .lineTo(new Vector2d(35,-12))       //park
                                //end parking

                                //end of program

                                .waitSeconds(2)

                                .build()

                )
                ;



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}
