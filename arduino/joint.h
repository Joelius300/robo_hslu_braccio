#ifndef JOINT_H_INCLUDED
#define JOINT_H_INCLUDED



//  arduino

    #include <Servo.h>
    #include <WString.h>



class Joint
{
        String Name;
        char Cmd;
        int Pin, Mini, Maxi;

        Servo Srv;
        int Current, Target;


    public:

        Joint (const String & name, char cmd, int pin, int mini, int maxi, int val);

        void init ();
        bool exec (char cmd, int val);
        void move ();
        void print () const;

};



#endif
