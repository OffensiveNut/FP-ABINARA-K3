#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from interpolation import interpolate
import serial
import threading

# Sesuaikan parameter berikut
PORT = '/dev/ttyACM0'  # Ganti dengan port yang sesuai
BAUDRATE = 9600

def send_serial(data_str):
    """
    Fungsi untuk mengirimkan data ke STM32.
    """
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            ser.write(data_str.encode())
    except serial.SerialException as e:
        print(f"Error sending data: {e}")

def receive_serial():
    """
    Fungsi untuk menerima data dari STM32.
    """
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"Listening on {PORT} at {BAUDRATE} baud...")
            while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').strip()
                    print(f"Data received: {data}")
    except serial.SerialException as e:
        print(f"Error receiving data: {e}")
    except KeyboardInterrupt:
        print("\nReceive thread terminated by user.")

def callback(data):
    """
    Callback function for receiving angles from ROS and sending to STM32.
    """
    # Retrieve the received array from the message
    angles = data.data  # data.data will contain the list of angles

    # Convert angles to digital units using interpolation
    digital_units = [interpolate(angle) for angle in angles]  # Use the interpolate function

    # Display the angles and their corresponding digital values
    rospy.loginfo(f'Sudut: {angles}')
    rospy.loginfo(f'Nilai: {digital_units}')

    # Format the digital units into a space-separated string, with each value padded to 4 digits
    formatted_units = ' '.join(f'{value:04d}' for value in digital_units)

    # Send the formatted string via serial
    send_serial(formatted_units)

def listener():
    """
    Initialize the ROS node and subscribe to the "servo" topic.
    """
    # Initialize the ROS node
    rospy.init_node('communicator', anonymous=True)

    # Create a subscriber to the "servo" topic, expecting messages of type Int32MultiArray
    rospy.Subscriber('servo', Int32MultiArray, callback)

    # Spin to keep the node running and processing messages
    rospy.spin()

def main():
    # Start a background thread to listen for incoming serial data
    receive_thread = threading.Thread(target=receive_serial, daemon=True)
    receive_thread.start()

    # Start the ROS listener
    listener()

if __name__ == '__main__':
    # Start the main function
    main()
