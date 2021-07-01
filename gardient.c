#include "kilolib.h"

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define TOO_CLOSE 40
#define DESIRED_DISTANCE 60
#define NEIGHBOUR_DIST 60

#define seedID 0

int current_motion = STOP;

int message_hop = 0;
int total_message_hop = 8;

int new_message = 0;
float new_dist = 0;
int min_dist = 200;

message_t message;
uint32_t last_update = 0;

int my_id = 7;

int myGradient = 255;
int received_gradient = 0;
int minGradient = 255;


float dist = 9000;

// Function to handle motion.
void set_motion(int new_motion)
{
    // Only take an an action if the motion is being changed.
    if (current_motion != new_motion)
    {
        current_motion = new_motion;
        
        if (current_motion == STOP)
        {
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == LEFT)
        {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == RIGHT)
        {
            spinup_motors();
            set_motors(0, kilo_turn_left);
        }
    }
}




void calculateGradient(int aux_gradient)
{
	
	if (myGradient > aux_gradient + 1)
	{
		myGradient = aux_gradient + 1;
		message.type = NORMAL;
		message.data[1] = myGradient;
		message.crc = message_crc(&message);
	}
}

void setup() {
    // put your setup code here, will be run once at the beginning
	message.type = NORMAL;
	message.data[1] = 0;
	message.crc = message_crc(&message);

	if( kilo_uid == 0 )
	{
		myGradient = 0;
		my_id = 0;
	}
}

void loop() 
{  
    if (new_message == 1 && my_id != seedID)
    {
	
		new_message = 0;
		message_hop++;
		last_update = kilo_ticks;

		if(received_gradient < minGradient && new_dist <= NEIGHBOUR_DIST)
		{
			minGradient = received_gradient;
			
		}

		if(message_hop == total_message_hop)
		{
			message_hop = 0;				

			myGradient = minGradient + 1;
			message.type = NORMAL;
			message.data[1] = myGradient;
			message.crc = message_crc(&message);

			
			minGradient = 255;
		}
	}

	if( myGradient == 0 )
	{
		set_color(RGB(0,1,0));
	}
	else if (myGradient == 1)
	{
		set_color(RGB(1,1,0));
	}
	else if (myGradient == 2)
	{
		set_color(RGB(0,0,1));
	}
	else if (myGradient == 3)
	{
		set_color(RGB(1,0,0));
	}
	else
	{
		set_color(RGB(0,0,0));
	}

	if( kilo_ticks > (last_update + 64) && myGradient < 255 && my_id != seedID)
	{
		set_motion(STOP);
		set_color(RGB(0,0,0));
		myGradient = myGradient + 1;
	}
 
}

message_t* message_tx()
{ 
	return &message;
} 


void message_rx(message_t *m, distance_measurement_t *d)
{
    new_message = 1;
    new_dist = estimate_distance(d);
    received_gradient = m->data[1];

}


int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;

	
    kilo_start(setup, loop);

    return 0;
}
