import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.ioloop
import serial
import thread

ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
print(ser.name)

def readSerial():
		while True:
			try:
				data = ser.readline();
				# data = 0
				# print 'from Arduino: ', data
				# received from Arduino written to all WebSocket clients
				[con.write_message(data) for con in WebSocketHandler.connections]
			except Exception:
				import traceback
				print traceback.format_exc()


class WebSocketHandler(tornado.websocket.WebSocketHandler):
		connections = set()

		def check_origin(self, origin):
			return True

		def open(self):
				self.connections.add(self)
				self.set_nodelay(True);
				print 'new connection was opened'
				pass

		def on_message(self, message):
				print 'from WebSocket: ', message
				b = bytearray(message, 'utf-8')
				try:
					ser.write(b+"\n");     # received from WebSocket writen to arduino
				# ser.write("ff?\n")
				except Exception:
					import traceback
					print traceback.format_exc()

		def on_close(self):
				self.connections.remove(self)
				print 'connection closed'
				pass


class Application(tornado.web.Application):
	def __init__(self):
		handlers = [
			(r'/websocket', WebSocketHandler),
			(r'/(.*)', tornado.web.StaticFileHandler, {'path': './root'})
		]

		settings = {
			'template_path': 'templates'
		}
		tornado.web.Application.__init__(self, handlers, **settings)

if __name__ == '__main__':
	ser.flushInput()
	thread.start_new_thread(readSerial, ())
	ws_app = Application()
	server = tornado.httpserver.HTTPServer(ws_app)
	server.listen(9090)
	tornado.ioloop.IOLoop.instance().start()
