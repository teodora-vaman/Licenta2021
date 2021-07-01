#include "kilolib.h"

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define TOO_CLOSE 30
#define DESIRED_DISTANCE 50
#define NEIGHBOUR_DIST 60

#define seedID 0

#define WAIT 0
#define MOVING 1

int current_motion = STOP;

int message_hop = 0;
int total_message_hop = 8;

int new_message = 0;
float new_dist = 0;
int dist_min = 200;

message_t message;
uint32_t last_update = 0;

int my_id = 7;

int myGradient = 255;
int received_gradient = 0;
int minGradient = 255;
int maxGradient = 0;

float dist = 9000;

int status = WAIT;

int edge_wait = 0;

// Function to handle motion.
void set_motion(int new_motion)
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
		set_motors(255,0);
		delay(15);  
		//spinup_motors();  
		set_motors(kilo_turn_left, 0);
		delay(500);
		set_motors(0, 0);
	}
	else if (current_motion == RIGHT)
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
		set_motion(FORWARD);
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
void setup() {
	message.type = NORMAL;
	message.data[1] = 0;
	message.crc = message_crc(&message);

	if( my_id == seedID )
	{
		myGradient = 0;
	}
}

//the main code - will run repeatedly
void loop() 
{  
	//check for incoming messages
    if (new_message == 1 && my_id != seedID)
    {
		new_message = 0;
		message_hop++;
		last_update = kilo_ticks;

		//use the received information to determine the minimum and maximum gradient value
		//and also the minimum distance
		if(received_gradient < minGradient && new_dist <= NEIGHBOUR_DIST)
		{
			minGradient = received_gradient;
		}

		if(received_gradient > maxGradient && new_dist <= NEIGHBOUR_DIST)
		{
			maxGradient = received_gradient;
		}

		if (new_dist < dist_min)
            dist_min = new_dist;

		//check if the number of total messages has been reahced
		if(message_hop == total_message_hop)
		{
			message_hop = 0;				

			myGradient = minGradient + 1;
			message.type = NORMAL;
			message.data[1] = myGradient;
			message.crc = message_crc(&message);

			if( myGradient > maxGradient)
			{
				edge_wait++;
			}

			minGradient = 255;

			if(status == MOVING)
				orbit(dist_min);
            dist_min = 200;
		}
	}

	//if the robot had the maximum gradient in its region for one second it will enter the MOVING state
	if( edge_wait > 32 && status == WAIT )
		status = MOVING;

	//color coding depending on the gradient
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

	//if no more messages were received in the last two seconds the robot will stop any movement and increase its gradient
	if( kilo_ticks > (last_update + 64) && myGradient < 255 && my_id != seedID)
	{
		set_motion(STOP);
		set_color(RGB(0,0,0));
		myGradient = myGradient + 1;
	}
 
}

//function for transmiting a message
message_t* message_tx()
{ 
	return &message;
} 

//function for receiving messages
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
