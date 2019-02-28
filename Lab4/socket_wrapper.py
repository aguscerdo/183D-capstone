from ws4py.client.threadedclient import WebSocketClient
import socket

class SocketWrapper:
	def __init__(self):
		sock = socket.gethostbyname(socket.gethostname())

		# self.addr = "ws://{}:81/ws".format(sock)
		self.addr = "ws://192.168.4.1:81/ws"
		self.socket = WebSocketClient(self.addr)

		self.socket.connect()
		print('Socket connected')


	def send_motion(self, u_left, u_right):
		payload = [126, int(u_left), int(u_right)]
		payload = bytearray(payload)

		self.socket.send(bytearray(payload), True)
		print('Payload sent: {}'.format(payload))


	def close(self):
		self.close()

