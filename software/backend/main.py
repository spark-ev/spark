import sys
#import rospy
from flask import Flask, jsonify, request
from gevent.pywsgi import WSGIServer
from flask_cors import CORS
#from std_msgs.msg import Float32, String

app = Flask(__name__)
CORS(app)

websocket_port = sys.argv[1]

# Initialize ROS node
#rospy.init_node('flask_ros_bridge', anonymous=True)
#speed_publisher = rospy.Publisher('/car/speed', Float32, queue_size=10)
#light_publisher = rospy.Publisher('/car/lights', String, queue_size=10)

@app.route('/api/actuators', methods=['GET']) # 100 Hz Polling
def actuators():
	actuators = {
		"throttle": 0, # 0-100% throttle
		"direction": "forward", # forward, reverse
		"brake": 0, # 0-100% brake
		"steering": 0, # -180 (absolute left) to 180 (absolute right)
	}
	return jsonify(actuators)

@app.route('/api/critical_sensors', methods=['GET']) # 50 Hz Polling
def critical_sensors():
    critical_sensors = {
        "accelerometer": [ # m/s^2
            {"id": "x", "value": 0},
			{"id": "y", "value": 0},
			{"id": "z", "value": 0}
		],
		"gyroscope": [ # Degrees
			{"id": "pitch", "value": 0},
			{"id": "yaw", "value": 0},
			{"id": "roll", "value": 0}
		],
		"temperatures": [ # Celsius
			{"id": "driveMotor", "temperature": 25},
			{"id": "batteryPack", "temperature": 25},
			{"id": "motorController", "temperature": 25},
			{"id": "exterior", "temperature": 25},
			{"id": "coolant", "temperature": 25},
		],
		"speed": [ # RPM
			{"id": "frontLeft", "value": 0},
			{"id": "frontRight", "value": 0},
			{"id": "backLeft", "value": 0},
			{"id": "backRight", "value": 0}
		]
	}
    return jsonify(critical_sensors)

@app.route('/api/sensors', methods=['GET']) # 25 Hz Polling
def sensors():
	sensors = {
		"chargeLevel": 100, # 0-100%
		"chassisSpeed": 20, # MPH
		"proximity": [
			{"id": 1, "distance": 10},
			{"id": 2, "distance": 20},
			{"id": 3, "distance": 30},
			{"id": 4, "distance": 40}
		]
	}
	return jsonify(sensors)

@app.route('/api/inputs', methods=['GET']) # 25 Hz Polling
def inputs():
	inputs = {
		"horn": 0, # 0, 1
		"lights": 0, # 0, 1
		"wipers": 0, # 0, 1
		"leftSignal": 0, # 0, 1
		"rightSignal": 0 # 0, 1
	}
	return jsonify(inputs)

@app.route('/api/streams', methods=['GET']) # 15 Hz Polling
def stream():
    stream = {
        "cameras": [
			{"id": 1, "name": "Front Camera", "data": ""},
			{"id": 2, "name": "Rear Camera", "data": ""},
			{"id": 3, "name": "Left Camera", "data": ""},
			{"id": 4, "name": "Right Camera", "data": ""}
		],
        "microphones": [
			{"id": 1, "name": "Passenger Microphone", "data": ""},
			{"id": 2, "name": "Driver Microphone", "data": ""}
		]
	}
    return jsonify(stream)

@app.route('/api/status', methods=['GET']) # 15 Hz Polling
def status():
    status = {
        "status": "active", # active, inactive
        "mode": "manual", # manual, autonomous
    }
    return jsonify(status)

#@app.route('/api/update_speed', methods=['POST'])
#def update_speed():
#    data = request.get_json()
#    speed = data.get('speed', 0.0)
#    speed_publisher.publish(speed)
#   response = {"status": "success", "speed": speed}
#    return jsonify(response)

#@app.route('/api/update_lights', methods=['POST'])
#def update_lights():
#    data = request.get_json()
#    light_command = data.get('command', 'off')
#    light_publisher.publish(light_command)
#    response = {"status": "success", "command": light_command}
#    return jsonify(response)

if __name__ == '__main__':
    # Debug/Development
	#app.run(host='0.0.0.0', port=int(websocket_port))
	# Production
	http_server = WSGIServer(('', int(websocket_port)), app)
	http_server.serve_forever()