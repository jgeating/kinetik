import socket

def connect_to_arduino(server_ip, server_port):
    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect to the server
    s.connect((server_ip, server_port))
    
    # server needs to be sent data for some reason
    s.send("hello".encode())

    # Receive the response
    response = ""
    print("getting response...")
    while True:

        data = s.recv(1024)
        if not data:
            break
        response += data.decode()
        print("receiving data...", response)
    s.close()
    return response

if __name__ == "__main__":
    server_ip = '192.168.0.6'  # Arduino IP
    server_port = 80  # Arduino port
    data = connect_to_arduino(server_ip, server_port)
    print(data)