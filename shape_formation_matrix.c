#include "kilolib.h"
#include<math.h>
#include<stdbool.h>

#define MAXIM 11
#define INF 10000

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define TOO_CLOSE 40
#define DESIRED_DISTANCE 50
#define NEIGHBOUR_DIST 50

#define seedID 0

#define WAIT 0
#define MOVING 1
#define INSIDE_SHAPE 2
#define LOCAL 3

//declaring the variables used

message_t message;
uint32_t last_update = 0;
uint32_t new_message;
uint16_t message_hop = 0;
uint16_t total_message_hop = 6;

int status;
int current_motion = STOP;
int edge_wait = 0;

int dist;
int gradient;
int id;
int localised_flag;
int shape_index;

int received_dist;
int received_id;
int received_timestamp;
int received_gradient;
int received_shape_index;
int received_local_flag;

//data structure to encapsulate all the information from the neighbouring robots
struct Neighbour
{
	int dist;
	int gradient;
	int id;
	int timestamp;
	int localised_flag;
	uint8_t flag;
	int shape_index;
};

struct Neighbour neighbours[MAXIM];

//the matrix used for creating a house shape
int rowsShape = 5;
int colsShape = 4;
float shape_array[5][4] = { {3,1,2,1}, {4,1,2,1.4142}, 
							{2,1,5,1}, {6,1.4142,4,1}, {2, 1, 6, 1.41}
						 }; 


float distance_array[MAXIM] = {-1, -1, -1, -1, -1, -1, -1, -1,-1,-1,-1};

//function to handle the motion
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
		//spinup_motors();  
		set_motors(kilo_turn_left, 0);
	}
	else if (new_motion == RIGHT)
	{
		set_motors(0,255);
		delay(15);
		//spinup_motors(); 
		set_motors(0, kilo_turn_right);
	}

	delay(500);
	set_motors(0, 0);
}

//function for deciding the direction of orbiting
void orbit(const int d)
{
	if(d < 30)
	{
		set_color(RGB(1,0,0));
		set_motion(FORWARD);
	}

	else if( d > ( DESIRED_DISTANCE ) )
	{
		set_color(RGB(1,1,0));
		set_motion(RIGHT);
	}
	else if ( d < (DESIRED_DISTANCE))
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

//helper function to set the message before transmitting 
void set_message()
{
	if(status == LOCAL)
		localised_flag = 1;
	else
		localised_flag = -1;
	message.type = NORMAL;
	message.data[0] = kilo_uid;  //id
	//message.data[1] = kilo_ticks;   //timestamp
	message.data[2] = gradient;
	message.data[3] = shape_index;
	message.data[5] = localised_flag;

	message.crc = message_crc(&message);
}

//helper function to set the elements in the neighbours array
void set_neighbour()
{
	neighbours[received_id].dist = received_dist;
	neighbours[received_id].flag = 1;
	neighbours[received_id].gradient = received_gradient;
	neighbours[received_id].localised_flag = received_local_flag;
	neighbours[received_id].timestamp = kilo_ticks;
	neighbours[received_id].shape_index = received_shape_index;
}

//helper function to discard any robot with whom he could not communicate in the last 3 seconds
void eliminateNeighbour()
{
	for(uint8_t i = 0; i < MAXIM; i++)
	{
		if(kilo_ticks > (neighbours[i].timestamp + 96) )
		{
			neighbours[i].dist = -1;
			neighbours[i].flag = -1;
			neighbours[i].gradient = -1;
			neighbours[i].localised_flag = -1;
			neighbours[i].timestamp = -1;
			neighbours[i].shape_index = 1000;
		}
	}
}

//data structure to encapsulate the concept of a point in space
struct point
{
	float x;
	float y;
	int dist;
};

void setup() 
{
	for(uint8_t i = 0; i < MAXIM; i++)
	{
		neighbours[i].flag = -1;
		neighbours[i].dist = -1;
		neighbours[i].timestamp = -1;
		neighbours[i].gradient = -1;
		neighbours[i].shape_index = 1000;
	}

	status = WAIT;
	if(kilo_uid <= 3)
	{
		gradient = 0;
		localised_flag = 1;
		status = LOCAL;
	}
	else
	{
		gradient = 255;
		localised_flag = 0;
	}
	
	switch (kilo_uid)
	{
	case 0:
		shape_index = 0;
		break;
	case 1:
		shape_index = 1;
		break;
	case 2:
		shape_index = 2;
		break;
	case 3:
		shape_index = 3;
		break;
	
	default:
		shape_index = 1000;
	}

	set_message();
}

void loop() 
{  
	//check for incoming messages and update the neighbours array
	if(new_message == 1)
	{
		new_message = 0;
		last_update = kilo_ticks;
		set_neighbour();
		if(received_local_flag == 1)
		{
			distance_array[received_shape_index] = received_dist;
		}
		message_hop++;
	}

	//check if the number of total messages has been reahced
	if(message_hop == total_message_hop)
	{
		message_hop = 0;
		int dist_min = 200;
		int gradient_min = 254;
		int gradient_max = 0;
		
		int valid = 0;
		struct point aux[4];
		int index = 0;
		for(uint8_t i = 0; i < MAXIM; i++)
		{
			if(neighbours[i].dist != -1 && neighbours[i].dist < dist_min)
				dist_min = neighbours[i].dist;
			if(neighbours[i].dist < 55 && neighbours[i].gradient != -1 && neighbours[i].gradient < gradient_min)
				gradient_min = neighbours[i].gradient;
			if(neighbours[i].dist < 55 && neighbours[i].gradient != -1 && neighbours[i].gradient > gradient_max)
				gradient_max = neighbours[i].gradient;
		}

		//update the gradient
		if(kilo_uid > 3)
			gradient = gradient_min + 1;

		//edge detection part
		if(kilo_uid > 3 && gradient > gradient_max)
			edge_wait++;
		else if(kilo_uid > 3)
			edge_wait = 0;
		
		if(edge_wait > 32 && status == WAIT)
			status = MOVING;

		//update the message that will be sent to other robots
		set_message();

		// switch (gradient)
		// {
		// case 0:
		// 	set_color(RGB(0,1,1));
		// 	break;
		// case 1:
		// 	set_color(RGB(1,1,0));
		// 	break;
		// case 2:
		// 	set_color(RGB(0,0,1));
		// 	break;
		// case 3:
		// 	set_color(RGB(1,0,0));
		// 	break;
		// case 4:
		// 	set_color(RGB(1,0,1));
		// 	break;
		// case 5:
		// 	set_color(RGB(0,1,0));
		// 	break;
		
		// default:
		// 	set_color(RGB(0,0,0));
		// 	break;
		// }

		//depending on the status of the robot either orbit around the group or change the color of the led
		if(kilo_uid >= 3 && dist_min < 200 && (status == MOVING || status == INSIDE_SHAPE))
			orbit(dist_min);
		else
			set_motion(STOP);

		if(status == LOCAL)
			set_color(RGB(3,0,3));
		
		if(status == INSIDE_SHAPE)
			set_color(RGB(0,3,3));

		//check for a valid position in the shape
		if (kilo_uid > 3 && status != LOCAL)
		{
			float aux_dist;
			int index;
			for(int i = 0; i < rowsShape; i++)
			{
				char ok = 1;
				for(int j = 0; j < colsShape; j = j + 2)
				{
					aux_dist = shape_array[i][j + 1] * 50;
					index = shape_array[i][j];
					if( (distance_array[index] < ( aux_dist - 10 )) || (distance_array[index] > ( aux_dist + 10 )))
						ok = 0;
				}

				if(ok == 1)
				{
					shape_index = i + 4;
					status = LOCAL;
					localised_flag = 1;
					orbit(dist_min);
					set_message();

					break;
				}
			}
		}
	}
	eliminateNeighbour();
}

//function for sending messages
message_t* message_tx()
{ 
	return &message;
} 

//function for receiving messages
void message_rx(message_t *m, distance_measurement_t *d)
{
    new_message = 1;
    received_dist = estimate_distance(d);
    received_id = m->data[0];
	//received_timestamp = m->data[1];
	received_gradient = m->data[2];
	received_shape_index = m->data[3];
	received_local_flag = m->data[5];
}

int main() 
{
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;

    kilo_start(setup, loop);

    return 0;
}
