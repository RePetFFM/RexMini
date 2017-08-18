import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.ioloop
import serial
import thread

ser = serial.Serial('/dev/ttyS0', 115200)
print(ser.name)

def readSerial():
		while True:
				data = ser.read();
				# print 'from Arduino: ', data
				# received from Arduino written to all WebSocket clients
				[con.write_message(data) for con in WebSocketHandler.connections]


class WebSocketHandler(tornado.websocket.WebSocketHandler):
		connections = set()

		def check_origin(self, origin):
			return True

		def open(self):
				self.connections.add(self)
				print 'new connection was opened'
				pass

		def on_message(self, message):
				print 'from WebSocket: ', message
				ser.write(message.encode('utf-8'));     # received from WebSocket writen to arduino

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