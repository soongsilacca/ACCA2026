import socket

# 서버의 주소입니다. hostname 또는 ip address를 사용할 수 있습니다.
HOST = 'www.gnssdata.or.kr'
# 서버에서 지정해 놓은 포트 번호입니다. 
PORT = 2101

# HOST = 'RTS2.ngii.go.kr'

# 소켓 객체를 생성합니다. 
# 주소 체계(address family)로 IPv4, 소켓 타입으로 TCP 사용합니다.  
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 지정한 HOST와 PORT를 사용하여 서버에 접속합니다. \
# connect_ex vs connect => https://stackoverflow.com/questions/48318266/python-socket-connect-vs-connect-ex
client_socket.connect_ex((HOST, PORT))
print("Socket Connected!...")

mountPointString = (
    "GET %s HTTP/1.0\r\nUser-Agent: %s\r\nAccept: */*\r\nAuthorization: Basic %s\r\nConnection: close\r\n"
    % ("/", "NTRIP u-blox", "tge1375@naver.com:gnss")
    )

mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
mountPointString += "\r\n"
mountPointString = mountPointString.encode()

client_socket.settimeout(10)
client_socket.send(mountPointString)

buffer = ''
data = ''
mountpoints = []

while True:
    buffer = client_socket.recv(4096)

    if not buffer: 
        break

    data = str(buffer, 'utf-8')

    if 'STR;' in data:
        data = data.split('\r\n')

    for line in data:
        if 'STR;' in line:
            mountpoints.append((line.split(';'))[1])

    # buffer += (str(data, 'utf-8'))

print(f"Available mountpoints list : ")
[print(point) for point in mountpoints]

print("We'll Use [FKP-RTCM31]")

RTCMString = (
    "GET %s HTTP/1.0\r\nUser-Agent: %s\r\nAccept: */*\r\nAuthorization: Basic %s\r\nConnection: close\r\n"
    % ("/FKP-RTCM31", "NTRIP u-blox", "tge1375@naver.com:gnss")
    )

RTCMString += "Ntrip-Version: Ntrip/2.0\r\n"
RTCMString += "\r\n"
RTCMString = RTCMString.encode()

client_socket.send(RTCMString)
# while True:
buffer = client_socket.recv(4096)
print(buffer)


# 소켓을 닫습니다.
client_socket.close()