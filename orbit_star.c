#include "kilolib.h"

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define ID 2


int current_motion = STOP;
uint32_t last_motion_update = 0;



message_t message;
// Flag to keep track of message transmission.
int message_sent = 0;

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
            set_motors(0, kilo_turn_right);
        }
    }
}



void setup() {
    // put your setup code here, will be run once at the beginning
	message.type = NORMAL;
	message.data[0] = ID;
	message.crc = message_crc(&message);
}

void loop() 
{
   if (message_sent == 1)
   {
        message_sent = 0;
        set_color(RGB(1,0,1));
        delay(100);
        set_color(RGB(0, 0, 0));
   }
}

message_t* message_tx()
{ 
	return &message;
} 


void message_rx(message_t *m, distance_measurement_t *d)
{
    //new_message = 1;
}

void message_tx_success()
{
    // Set flag on message transmission.
    message_sent = 1;
}


int main() {
    kilo_init();
    // Register the message_tx callback function.
    kilo_message_tx = message_tx;
    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;

	
    kilo_start(setup, loop);

    return 0;
}
