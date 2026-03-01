#include "dynamixel.h"
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>

void move_to_location(int connection, unsigned char id,
			unsigned char loc_h, unsigned char loc_l) {

	unsigned char cs = ~ ( id + 0x07 + 0x03 + 0x1e + loc_l + loc_h +
				0x30 + 0x00);

	unsigned char arr[] = { 0xff, 0xff, id, 0x07, 0x03, 0x1e, loc_l,
                                       loc_h, 0x30, 0x00, cs };

	int buff_len = 100;
	unsigned char buff[buff_len];

	int bytes_read = write_to_connection(connection,arr,11,buff,buff_len);

}

bool isMoving(int connection){
	/*
	Iterates through each motor and determines if is currently running
	If runnin return true, if no motors are running return false
	*/
	unsigned char len = 0x04;
    unsigned char instr = 0x02;
    unsigned char loc_l = 0x2e;
    unsigned char loc_h = 0x01;
	for(int i = 1; i < 6; i ++){
		unsigned char cs = ~ (i + len + instr + loc_l + loc_h);
		
		unsigned char arr[] = {0xff,0xff,i,len,instr,loc_l,loc_h,cs};
		int buff_len = 100;
		unsigned char buff[buff_len];

		int bytes_read = write_to_connection(connection,arr,8,buff,buff_len);
		if (buff[5] == 0x01){
			return true;
		}
	}
	return false;
		
}

void wait_until_done(int connection) {
	/*
	Calls is moving every 0.02 seconds, dealys the robot until no motor is running
	*/
	while (isMoving(connection)){
		usleep(20000);
	}

}
void grab(int connection){
	move_to_location(connection,5,0x01,0x37);
	wait_until_done(connection);
}
void release(int connection){
	move_to_location(connection,5,0x01,0xe1); // open claws
	wait_until_done(connection);
}

void pickUp(int connection,unsigned char arr[8], int position, bool rotateFirst){
	/*
	Provided with an array of 8 elements, each pair (e.g. 0,1 2,3) providing the low and high values for each motor
	rotateFirst determines order of execution to prevent blocks being knocked over
	grabs the block at the end
	position determines which stack picking up from (by altering motor 1 - the rotation motor)
	*/
	unsigned char first = arr[0];
	unsigned char second = arr[1];
	if (position == 2){
		first = 0x01;
		second = 0xfb;
	}else if(position == 3){
		first = 0x03;
		second = 0x1d;
	}

	if(rotateFirst){
		move_to_location(connection,1,first,second);
		wait_until_done(connection);
		move_to_location(connection,2,arr[2],arr[3]);
		move_to_location(connection,3,arr[4],arr[5]);
		move_to_location(connection,4,arr[6],arr[7]);
		wait_until_done(connection);		
	}else{
		move_to_location(connection,2,arr[2],arr[3]);
		move_to_location(connection,3,arr[4],arr[5]);
		move_to_location(connection,4,arr[6],arr[7]);
		wait_until_done(connection);
		move_to_location(connection,1,first,second);
		wait_until_done(connection);
	}
	grab(connection);
		
}

void dropOff(int connection,unsigned char arr[8], int position, bool rotateFirst){
	/*
	Provided with an array of 8 elements, each pair (e.g. 0,1 2,3) providing the low and high values for each motor
	rotateFirst determines order of execution to prevent blocks being knocked over
	Releases the block at the end
	position determines which stack dropping off at (by altering motor 1 - the rotation motor)
	*/
	unsigned char first = arr[0];
	unsigned char second = arr[1];
	if (position == 2){
		first = 0x01;
		second = 0xfb;
	}else if(position == 3){
		first = 0x03;
		second = 0x1d;
	}
	if(rotateFirst){
		move_to_location(connection,1,first,second);
		wait_until_done(connection);
		move_to_location(connection,2,arr[2],arr[3]);
		move_to_location(connection,3,arr[4],arr[5]);
		move_to_location(connection,4,arr[6],arr[7]);
		wait_until_done(connection);
	}else{		
		move_to_location(connection,2,arr[2],arr[3]);
		move_to_location(connection,3,arr[4],arr[5]);
		move_to_location(connection,4,arr[6],arr[7]);
		wait_until_done(connection);
		move_to_location(connection,1,first,second);
		wait_until_done(connection);
	}
	release(connection);
	
}
void middle(int connection){

	move_to_location(connection,2,0x01,0xff);
	move_to_location(connection,3,0x01,0xff);
	move_to_location(connection,4,0x01,0xff);
	wait_until_done(connection);
	move_to_location(connection,1,0x01,0xff);
	wait_until_done(connection);

}

int main(int argc, char* argv[]) {

	/*
	The main section of code initialises arrays and connections before completing the tower of hanoi algorithm
	by passing array values, stack position and order of actions
	middle is used to prevent collisions between the arm and stacked blocks when moving
	*/

	//define arrays that provide position for block

	unsigned char floorH[8] = {0x00,0xc1,0x01,0x19,0x01,0x5b,0x01,0x18};
	unsigned char middleH[8] = {0x00,0xc1,0x01,0x12,0x01,0xa3,0x00,0xcc};
	unsigned char maxH[8] = {0x00,0xc1,0x01,0x2e,0x01,0x94,0x00,0xc4};


	int connection = open_connection("/dev/ttyUSB0",B1000000);

	// TOWER OF HANOI ALGORITHM

	//first move

	move_to_location(connection,5,0x01,0xe1); // open claws
	wait_until_done(connection);

	middle(connection); // back to middle

	pickUp(connection,maxH,1,false); // pick up first block
	
	middle(connection); // back to middle

	dropOff(connection,floorH,3,true); //drop off first block at floor in 3rd stack

	//second move

	middle(connection); // back to middle

	pickUp(connection,middleH,1,false); // pick middle from stack 1

	middle(connection);

	dropOff(connection,floorH,2,true); //drop of on floor of stack 2

	//third move

	middle(connection); // back to middle

	pickUp(connection,floorH,3,true); // pick up floor from stack 3
	
	middle(connection); 

	dropOff(connection,middleH,2,true); // drop off middle in stack 2

	//fourth move

	middle(connection); // back to middle

	pickUp(connection,floorH,1,true); //pick up floor from stack 1
	
	middle(connection); 

	dropOff(connection,floorH,3,true); //drop off on floor of stack 3

	//fifth move

	middle(connection); // back to middle

	pickUp(connection,middleH,2,true); // pick up middle from stack 2
	
	middle(connection); 

	dropOff(connection,floorH,1,true); //place on floor of stack 1

	//sixth move

	middle(connection); // back to middle

	pickUp(connection,floorH,2,true); 
	
	middle(connection); 

	dropOff(connection,middleH,3,true); // place floor of stack 2 on middle of stack 3

	//seventh move

	middle(connection); // back to middle

	pickUp(connection,floorH,1,true); 
	
	middle(connection); 

	dropOff(connection,maxH,3,false); // place floor of stack 1 on top of stack 3

	return 0;

}
