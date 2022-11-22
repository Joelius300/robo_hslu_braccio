//  self

    #include "parser.h"


//  app

    #include "joint.h"
    #include "robot.h"


//  arduino

    #include <Arduino.h>



Parser :: Parser ()
    : Val (0)
{
}



void Parser :: exec (Robot & rob)
{
    if (Serial.available() <= 0)
        return;

    char chr = Serial.read();

    if ('0' <= chr && chr <= '9')
    {
        Val = 10 * Val + (chr - '0');
        return;
    }

    if (rob.exec (chr, Val))
    {
        rob.print();
        Val = 0;
    }
}
