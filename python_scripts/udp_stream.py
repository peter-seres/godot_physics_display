import socket
from .data_packet import DataPacket


class StreamUDP:
    def __init__(self, port: int, target_port: int):
        self.port = port
        self.target_address = ('127.0.0.1', target_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', self.port))

    def send(self, packet: DataPacket) -> None:
        message = packet.to_bytes()
        self.socket.sendto(message, self.target_address)
