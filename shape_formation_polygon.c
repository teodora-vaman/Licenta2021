#include "kilolib.h"
#include<math.h>
#include <stdio.h>
#include<stdbool.h>

#define MAXIM 11
#define INF 1000

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define TOO_CLOSE 35
#define DESIRED_DISTANCE 50
#define NEIGHBOUR_DIST 50

#define seedID 0

#define WAIT 0
#define MOVING 1
#define INSIDE_SHAPE 2
#define LOCAL 3

//structure for encapsulating the concept of a point
struct point
{
	int x;
	int y;
	int dist;
};

//data structure to encapsulate all the information from the neighbouring robots
struct Neighbour
{
	uint16_t dist;
	uint16_t gradient;
	uint16_t id;
	uint16_t timestamp;
	uint16_t localised_flag;
	uint8_t flag;
	uint16_t x;
	uint16_t y;
};

//declaring the variables used
struct point aux_point;

message_t message;
uint32_t last_update = 0;
uint32_t new_message;
uint16_t message_hop = 0;
uint16_t total_message_hop = 6;

uint16_t status;
uint16_t current_motion = STOP;
uint16_t edge_wait = 0;

uint16_t dist;
uint16_t gradient;
uint16_t id;
uint16_t localised_flag;
uint16_t x;
uint16_t y;

uint16_t received_dist;
uint16_t received_id;
uint16_t received_timestamp;
uint16_t received_gradient;
uint16_t received_x;
uint16_t received_y;
uint16_t received_local_flag;

//int index = 0;
uint16_t inside_shape;
uint16_t valid = 0;
struct point localised_neighbours[10];
struct Neighbour neighbours[MAXIM];

//the polygon in which the robots must assemble
struct point polygon[] = { { 125, 75 },{ 125, 275 },{ 275, 275 },{ 275, 75 } };
//polygon used to detect the moment the robots leave the shape
struct point polygon_exit[] = { { 115, 100 },{ 115, 220 },{ 275, 220 },{ 275, 100 } };


// function to handle motion
void set_motion(uint8_t new_motion)
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

//helper function to set the message before transmitting 
void set_message()
{
	message.type = NORMAL;
	message.data[0] = kilo_uid;  //id
	//message.data[1] = kilo_ticks;   //timestamp
	message.data[2] = gradient;
	message.data[3] = x;
	message.data[4] = y;
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
	neighbours[received_id].x = received_x;
	neighbours[received_id].y = received_y;
}

//helper function to discard any robot with whom he could not communicate in the last 3 seconds
void eliminateNeighbour()
{
	for(uint8_t i = 0; i < MAXIM; i++)
	{
		if(kilo_ticks > (neighbours[i].timestamp + 64) )
		{
			neighbours[i].dist = -1;
			neighbours[i].flag = -1;
			neighbours[i].gradient = -1;
			neighbours[i].localised_flag = -1;
			neighbours[i].timestamp = 0;
			neighbours[i].x = 1000;
			neighbours[i].y = 1000;
		}
	}

}

int max(int a, int b)
{
	return (a > b) ? a : b;
}

int min(int a, int b)
{
	return (a < b) ? a : b;
}

// -- TRILATERATION -- //
struct point trilateration(struct point p1, struct point p2, struct point p3)
{
	int A = -2 * p1.x + 2 * p2.x;
	int B = -2 * p1.y + 2 * p2.y;
	int C = p1.dist * p1.dist - p2.dist*p2.dist - p1.x*p1.x + p2.x * p2.x - p1.y*p1.y + p2.y * p2.y;

	int D = -2 * p2.x + 2 * p3.x;
	int E = -2 * p2.y + 2 * p3.y;
	int F = p2.dist * p2.dist - p3.dist*p3.dist - p2.x*p2.x + p3.x * p3.x - p2.y*p3.y + p3.y * p3.y;

	struct point aux;
	aux.x = (C*E - F*B) / (E*A - B*D);
	aux.y = (C*D - F*A) / (D*B - A*E);

	return aux;
}

// -- RAY CASTING -- //

//function to check if point q is inside the segment pr
uint8_t onSegment(struct point p, struct point q, struct point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
            return 1;
    return 0;
}

//function to determine the orientation of three points
// 0 : p q r are colinear
// 1 : p q r are in clockwise orientation
// 2 : p q r are in counter clockwise orientation
uint8_t orientation(struct point p, struct point q, struct point r)
{
	int val = (q.y - p.y) * (r.x - q.x) -
		(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0; // colinear
	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

//function to check is segments p1q1 and p2q2 intersect 
uint8_t doIntersect(struct point p1, struct point q1, struct point p2, struct point q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return 1;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return 1;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return 1;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return 1;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return 1;

	return 0; // Doesn't fall in any of the above cases
}


// Returns true if the point p lies inside the polygon[] with n vertices
uint8_t isInside(struct point polygon[], int n, struct point p)
{
	// There must be at least 3 vertices in the polygon
	if (n < 3) return 0;

	// Create a point for line segment from p to infinite
	struct point extreme = { INF, p.y };

	// Count intersections of the above line with sides of polygon
	uint8_t count = 0;
	uint8_t i = 0;
	do
	{
		int next = (i + 1) % n;

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(polygon[i], polygon[next], p, extreme))
		{
			// If the point 'p' is colinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(polygon[i], p, polygon[next]) == 0)
				return onSegment(polygon[i], p, polygon[next]);

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	if (count % 2 == 0)
		return 1;
	return 0;
}

void setup() 
{
	//initialize all the neigbours in the aray
	for(uint8_t i = 0; i < MAXIM; i++)
	{
		neighbours[i].flag = -1;
		neighbours[i].dist = -1;
		neighbours[i].timestamp = -1;
		neighbours[i].gradient = -1;
		neighbours[i].x = 1000;
		neighbours[i].y = 1000;
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
		x = INF;
		y = INF;
	}
	
	uint8_t mod_id = kilo_uid % 10;
	switch (mod_id)
	{
	case 0:
		x = 100;
		y = 100;
		break;
	case 1:
		x = 100;
		y = 150;
		break;
	case 2:
		x = 150;
		y = 150;
		break;
	case 3:
		x = 150;
		y = 100;
		break;
	
	default:
		x = 1000;
		y = 1000;
	}

	set_message();
}

void loop() 
{  
	//check for incoming messages and update the neigbours array
	if(new_message == 1)
	{
		new_message = 0;
		last_update = kilo_ticks;
		set_neighbour();
		message_hop++;
	}

	if(message_hop == total_message_hop)
	{
		message_hop = 0;
		uint8_t dist_min = 200;
		uint8_t gradient_min = 254;
		uint8_t gradient_max = 0;
		for(uint8_t i = 0; i < MAXIM; i++)
		{
			if(neighbours[i].dist != -1 && neighbours[i].dist < dist_min)
				dist_min = neighbours[i].dist;
			if(neighbours[i].dist < 55 && neighbours[i].gradient != -1 && neighbours[i].gradient < gradient_min)
				gradient_min = neighbours[i].gradient;
			if(neighbours[i].dist < 55 && neighbours[i].gradient != -1 && neighbours[i].gradient > gradient_max)
				gradient_max = neighbours[i].gradient;			
		}

		if(kilo_uid > 3)
			gradient = gradient_min + 1;


		//edge detection part to transition from the WAIT state to MOVING state
		if(kilo_uid > 3 && gradient > gradient_max)
			edge_wait++;
		else 
			edge_wait = 0;
		
		if(kilo_uid > 3 && edge_wait > 64 && status == WAIT)
			status = MOVING;

		//if already inside the shape check if the point left the are of the edge polygon
		//if so, change the status to LOCAL, meaning th elocal will be stabilized inside the shape and stop moving
		if (status == INSIDE_SHAPE)
		{
			if(x != 1000 && y!= 1000)
			{
				aux_point.x = x;
				aux_point.y = y;
				inside_shape = isInside(polygon_exit, 4, aux_point);

				if(inside_shape == 0)
				{
					status = LOCAL;
				}
			}

			for(uint8_t i = 0; i < MAXIM; i++)
				if(neighbours[i].gradient == gradient && neighbours[i].dist < 55 )
					status = LOCAL;

		}
		
		//check if at least three localized robots are in range
		valid = 0;
		for(uint8_t i = 0; i < MAXIM; i++)
		{
			if(neighbours[i].x != 1000 && neighbours[i].localised_flag == 1)
			{
				localised_neighbours[valid].x = neighbours[i].x;
				localised_neighbours[valid].y = neighbours[i].y;
				localised_neighbours[valid].dist = neighbours[i].dist;
				valid++;
			}
		}

		//use the stabilized neighbours to calculate postion and determine if inside the polygon
		if(kilo_uid > 3 && valid >= 3)
		{
			aux_point = trilateration(localised_neighbours[0], localised_neighbours[1], localised_neighbours[2]);
			x = aux_point.x;
			y = aux_point.y;
			
			if(status == MOVING)
			{
				inside_shape = isInside(polygon, 4, aux_point);
				if(inside_shape == 1)
				{
					status = INSIDE_SHAPE;
					set_color(RGB(1,1,1));
				}
				else
					set_color(RGB(1,0,0));
			}
		}
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
			set_color(RGB(1,0,1));
		
		if(status == INSIDE_SHAPE)
			set_color(RGB(0,1,1));
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
	received_x = m->data[3];
	received_y = m->data[4];
	received_local_flag = m->data[5];

	//received_local_flag = m->data[2];

}

int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;

    kilo_start(setup, loop);

    return 0;
}
