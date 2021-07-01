#include "kilolib.h"

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define TOO_CLOSE 30
#define DESIRED_DISTANCE 50

message_t message;
int message_hop = 0;
int total_msg_hop = 6;

int new_message;
int new_dist;
int dist_min;

// Function to handle motion.
void set_motion(int new_motion)
{
    if (new_motion == STOP)
    {
        set_motors(0, 0);
    }
    else if (new_motion == FORWARD)
    {
        spinup_motors();
        set_motors(kilo_straight_left, kilo_straight_right);
    }
    else if (new_motion == LEFT)
    {
        set_motors(255,0);
        delay(15);  
        //spinup_motors(); //this function starts the both motors for 15ms 
        set_motors(kilo_turn_left, 0);
        delay(500);
        set_motors(0, 0);
    }
    else if (new_motion == RIGHT)
    {
        set_motors(0,255);
        delay(15);
        //spinup_motors(); 
        set_motors(0, kilo_turn_right);
        delay(500);
        set_motors(0, 0);
    }
}

//function for deciding the direction of orbiting
void orbit(const int d)
{
    if(d < TOO_CLOSE)
    {
        set_color(RGB(1,0,0));
        set_motion(STOP);
    }

    else if( d > DESIRED_DISTANCE )
    {
        set_color(RGB(1,1,0));
        set_motion(RIGHT);
    }
    else if ( d < DESIRED_DISTANCE )
    {
        set_color(RGB(0,0,1));
        set_motion(LEFT);
    }
    else
    {
        set_color(RGB(1,0,0));
        set_motion(FORWARD);
    }
}

//setup function that runs only once
//used for inisialisation
void setup() 
{
	dist_min = 200;
	new_dist = 0;
	new_message = 0;
	message_hop = 0;
}

//the main code - will run repeatedly
void loop() 
{
	//check if a new message arrived
    if (new_message == 1)
    {
        new_message = 0;
        message_hop++;

		//remember the minimum distance
        if (new_dist < dist_min)
            dist_min = new_dist;

		//when the total number of messages has been reached 
		//begin the orbiting process
        if(message_hop == total_msg_hop)
        {
            message_hop = 0;
            orbit(dist_min);
            dist_min = 200;
        }     
    }
}

//function for receiveing messages
void message_rx(message_t *m, distance_measurement_t *d)
{
    new_message = 1;
    new_dist = estimate_distance(d);
}

int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);
    return 0;
}
