import BT  # Import the Bluetooth module for communication
import time  # Import the time module for delays
import MotoriAI  # Import the MotoriAI module for motor control and AI
import Led  # Import the Led module for LED control

Led.lightBlue()  # Turn on the blue LED
BT.wait_for_response("V", 70)  # Wait for a 'V' response from Bluetooth with a timeout of 70 seconds
MotoriAI.closeDoor(0.4)  # Close the door
BT.wait_for_response("O", 50)  # Wait for an 'O' response from Bluetooth with a timeout of 50 seconds
MotoriAI.openDoor(0.5)  # Open the door
time.sleep(1)  # Wait for 1 second
MotoriAI.straigthAI(10)  # Move straight ahead for 10 seconds with AI
time.sleep(2)  # Wait for 2 seconds
MotoriAI.left(2)  # Turn left for 2 seconds
MotoriAI.leftAI(2.7)  # Turn left with AI assistance for 2.7 seconds
MotoriAI.stop()  # Stop all motor activity
time.sleep(2)  # Wait for 2 seconds
BT.walking_for_response("S", 4)  # Wait for an 'S' response from Bluetooth while walking for 4 seconds
MotoriAI.stop()  # Stop all motor activity again
time.sleep(10)  # Wait for 10 seconds
MotoriAI.straigthAI(26)  # Move straight ahead for 26 seconds with AI
MotoriAI.stop()  # Stop all motor activity
time.sleep(2)  # Wait for 2 seconds
MotoriAI.closeDoor(1)  # Close the door with a 1 second delay
time.sleep(2)  # Wait for 2 seconds
BT.send_mess("U")  # Send a 'U' message via Bluetooth
print("Motori spenti")  # Print "Motors off" to indicate the end of the operation
Led.turnOff()  # Turn off the LED