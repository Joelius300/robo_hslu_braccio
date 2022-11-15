#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED



//  app

    #include "joint.h"



class Robot
{
        Joint Jnt [6];

        void SoftPWM (int high_time, int low_time);
        void SoftStart ();


    public:

        Robot ();

        void init ();
        bool exec (char cmd, int val);
        void move ();
        void print () const;
};



#endif
