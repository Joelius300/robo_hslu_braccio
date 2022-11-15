#include "parser.h"
#include "robot.h"



Robot Rob;
Parser Prs;



void setup ()
{
    Serial.begin (9600);

    Serial.println ("initializing, please wait ...");
    Rob.init();
    Serial.println ("done");

    Rob.print();
}



void loop ()
{
    Prs.exec (Rob);
    Rob.move();
    delay (20);
}
