import serial
import time
import MotoriAI

# Serial port configuration
uart_ch_gv = serial.Serial("/dev/ttyAMA0", baudrate=38400, timeout=1)
 
# Function for sending a message to the robot
def send_mess(mess):
    msg=""
    ricevuto = False
    timeout = False
    encode_mess=mess.encode('UTF-8')
    while not ricevuto:
        # sends the message
        uart_ch_gv.write(encode_mess)
        time.sleep(1)
        t0 = time.time()
        print(mess)
        while not ricevuto and not timeout:
            # reading the response
            data = uart_ch_gv.read(1)
            data1 = data.decode('UTF-8', 'ignore')
            if data1 != "":
                if data1 == "$":
                    msg = ""
                elif data1 == '#':
                    # if the response is ACK, it mean that the message was received correctly
                    print(msg)
                    if msg == "ACK":
                        #Led.yellow()   
                        print("ho ricevuto ACK")
                        ricevuto = True
                elif data1 >= "A" and data1 <="Z":
                    msg += data1
            uart_ch_gv.flush()
            t1 = time.time()
            # timeout di 5 secondi // 5 seconds timeout
            if not ricevuto and (t1 - t0) > 5:
                timeout = True
                break

# function for reading data off the sensor and for moving the robot
def sensor_read_prova():
    # initializaton of the serial connection with the sensor
    uart_channel = serial.Serial("/dev/ttyAMA0", baudrate=38400, timeout=1)
    data=b''
    t0=time.time()
    distance=89
    while distance > 70 and time.time()-t0 < 3.6:
        distance_str = ""
        data1 = ""
        while data1 != "!" and time.time()-t0 < 3.6:
            data = uart_channel.read(1)
            data1 = data.decode('UTF-8')
            distance_str += data1
        distance = int(distance_str[:-1])
        print(distance)
        print(time.time()-t0)
   
    print("Girare")
    # stop the serial communciation
    uart_ch_gv.close()
    uart_channel.close()

def wait_for_response(expected_response, timeout_seconds):
    msg = ""
    ricevuto = False
    t0 = time.time()

    while not ricevuto and (time.time() - t0) < timeout_seconds:
        # Answer reading
        data = uart_ch_gv.read(1)
        data1 = data.decode('UTF-8', 'ignore')

        if data1 != "":
            if data1 == "$":
                msg = ""
            elif data1 == '#':
                # if the response is ACK, it mean that the message was received correctly
                print(msg)
                if msg == expected_response:
                    print(f"ho ricevuto {expected_response}")
                    ricevuto = True
            elif data1 >= "A" and data1 <= "Z":
                msg += data1

        uart_ch_gv.flush()

    # if the timeout is expired without receiving the expected response
    if not ricevuto:
        print(f"Timeout ({timeout_seconds} secondi) senza ricevere la risposta attesa ({expected_response})")
        return    
    

def walking_for_response(expected_response, timeout_seconds):
    msg = ""
    ricevuto = False
    t0 = time.time()
    MotoriAI.straight()
    print("Motori partiti")
    while not ricevuto and (time.time() - t0) < timeout_seconds:
        # Answer reading
        data = uart_ch_gv.read(1)
        data1 = data.decode('UTF-8', 'ignore')

        if data1 != "":
            if data1 == "$":
                msg = ""
            elif data1 == '#':
                # if the response is ACK, it mean that the message was received correctly
                print(msg)
                if msg == expected_response:
                    print(f"ho ricevuto {expected_response}")
                    ricevuto = True
            elif data1 >= "A" and data1 <= "Z":
                msg += data1
        uart_ch_gv.flush()

    # if the timeout is expired without receiving the expected response
    if not ricevuto:
        print(f"Timeout ({timeout_seconds} secondi) senza ricevere la risposta attesa ({expected_response})")
        return        
