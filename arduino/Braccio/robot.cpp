//  self

    #include "robot.h"


//  app

    #include "parser.h"


//  arduino

    #include <Arduino.h>



#define SOFT_START_PIN 12



Robot :: Robot ():
    Jnt
    {
        { "base          ", 'b', 11,  0, 360, 90 },
        { "shoulder      ", 's', 10, 15, 165, 90 },
        { "elbow         ", 'e',  9,  0, 180, 90 },
        { "wrist flexion ", 'v',  6,  0, 180, 90 },
        { "wrist rotation", 'w',  5,  0, 180, 90 },
        { "gripper       ", 'g',  3, 10,  73, 60 }
    }
{
}



void Robot :: SoftPWM (int high_time, int low_time)
{
    digitalWrite (SOFT_START_PIN, HIGH);
    delayMicroseconds (high_time);
    digitalWrite (SOFT_START_PIN, LOW);
    delayMicroseconds (low_time);
}



void Robot :: SoftStart ()
{
    long int tmp = millis();

    while (millis() - tmp < 2000)
      SoftPWM (80, 450);

    while (millis() - tmp < 6000)
      SoftPWM (75, 430);

    digitalWrite (SOFT_START_PIN, HIGH);
}



void Robot :: init ()
{
    for (Joint & j: Jnt)
        j.init();
    SoftStart();
}



void Robot :: move ()
{
    for (Joint & j: Jnt)
        j.move();
}



bool Robot :: exec (char cmd, int val)
{
    for (Joint & j: Jnt)
        if (j.exec (cmd, val))
            return true;

    return false;
}



void Robot :: print () const
{
    Serial.println();
    for (const Joint & j: Jnt)
        j.print();
}
