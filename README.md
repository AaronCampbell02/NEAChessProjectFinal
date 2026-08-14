Robot arm tower of Hanoi solution
This is a C program that moves a robot arm to pick and place 3 stacked blocks to solve the tower of Hanoi using an optimised 7 move solution

Hardware
Dynamixel servo-driven robot arm (5 motors: base rotation + 3 arm joints + claw)
Connects via serial (/dev/ttyUSB0, baud rate 1,000,000)

Built based of abstractions to produce a library to make the code more extendable in future:

move_to_location — sends a positioning packet to a specific motor using a checksum to verify.
isMoving — polls all 5 motors to check if any are still in motion.
wait_until_done — blocks until all motors have stopped moving before issuing the next command.
grab / release — opens/closes the claw motor to pick up or drop a block.
pickUp / dropOff — moves the arm to a given stack position and picks up/drops off a block.
middle — returns the arm to a safe position in the middle.
main — defines the 3 block-height position arrays (floor/middle/max), opens the connection to the arm, and executes the 7 moves.

