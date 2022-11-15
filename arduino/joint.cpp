//  self

    #include "joint.h"


//  arduino

    #include <Arduino.h>



Joint :: Joint (const String & name, char cmd, int pin, int mini, int maxi, int val)
    : Name    (name)
    , Cmd     (cmd )
    , Pin     (pin )
    , Mini    (mini)
    , Maxi    (maxi)
    , Current (val )
    , Target  (val )
{
}



void Joint :: init ()
{
    Srv.attach (Pin, 544, 2400);
}



bool Joint :: exec (char cmd, int val)
{
    if (cmd != Cmd)
        return false;

    if (val < Mini) val = Mini;
    if (val > Maxi) val = Maxi;
    Target = val;
    return true;
}



void Joint :: move ()
{
    if (Current != Target)
    {
        int delta = Current > Target ? -1 : 1;
        Current += delta;
        Srv.write (Current);
    }
}



void Joint :: print () const
{
    Serial.print (Cmd);
    Serial.print (": ");
    Serial.print (Name);
    Serial.print (' ');
    Serial.println (Target);
}
