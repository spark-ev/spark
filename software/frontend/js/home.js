const axios = require("axios");
const { ipcRenderer } = require("electron");

function updatePageWithData(endpoint, data) {
	// Update the page with data based on the endpoint
	if (endpoint === "actuators") {
		document.getElementById("throttle").textContent = data.throttle;
		document.getElementById("direction").textContent = data.direction;
		document.getElementById("brake").textContent = data.brake;
		document.getElementById("steering").textContent = data.steering;
	} else if (endpoint === "critical_sensors") {
		document.getElementById("accelerometer-x").textContent =
			data.accelerometer[0].value;
		document.getElementById("accelerometer-y").textContent =
			data.accelerometer[1].value;
		document.getElementById("accelerometer-z").textContent =
			data.accelerometer[2].value;

		document.getElementById("gyroscope-pitch").textContent =
			data.gyroscope[0].value;
		document.getElementById("gyroscope-yaw").textContent =
			data.gyroscope[1].value;
		document.getElementById("gyroscope-roll").textContent =
			data.gyroscope[2].value;

		document.getElementById("temperature-driveMotor").textContent =
			data.temperatures[0].temperature;
		document.getElementById("temperature-batteryPack").textContent =
			data.temperatures[1].temperature;
		document.getElementById("temperature-motorController").textContent =
			data.temperatures[2].temperature;
		document.getElementById("temperature-exterior").textContent =
			data.temperatures[3].temperature;
		document.getElementById("temperature-coolant").textContent =
			data.temperatures[4].temperature;

		document.getElementById("speed-frontLeft").textContent =
			data.speed[0].value;
		document.getElementById("speed-frontRight").textContent =
			data.speed[1].value;
		document.getElementById("speed-backLeft").textContent =
			data.speed[2].value;
		document.getElementById("speed-backRight").textContent =
			data.speed[3].value;
	} else if (endpoint === "sensors") {
		document.getElementById("chargeLevel").textContent = data.chargeLevel;
		document.getElementById("chassisSpeed").textContent = data.chassisSpeed;

		document.getElementById("proximity-1").textContent =
			data.proximity[0].distance;
		document.getElementById("proximity-2").textContent =
			data.proximity[1].distance;
		document.getElementById("proximity-3").textContent =
			data.proximity[2].distance;
		document.getElementById("proximity-4").textContent =
			data.proximity[3].distance;
	} else if (endpoint === "inputs") {
		document.getElementById("horn").textContent = data.horn;
		document.getElementById("lights").textContent = data.lights;
		document.getElementById("wipers").textContent = data.wipers;
		document.getElementById("leftSignal").textContent = data.leftSignal;
		document.getElementById("rightSignal").textContent = data.rightSignal;
	} else if (endpoint === "streams") {
		// Update stream data
	} else if (endpoint === "status") {
		document.getElementById("status").textContent = data.status;
		document.getElementById("mode").textContent = data.mode;
	}
}

document.addEventListener("DOMContentLoaded", () => {
	let websocketPort;

	const intervals = {
		actuators: 10, // 100 Hz
		critical_sensors: 20, // 50 Hz
		sensors: 40, // 25 Hz
		inputs: 40, // 25 Hz
		streams: 67, // 15 Hz
		status: 67, // 15 Hz
	};

	let lastTimestamps = {
		actuators: 0,
		critical_sensors: 0,
		sensors: 0,
		inputs: 0,
		streams: 0,
		status: 0,
	};

	async function fetchData(endpoint) {
		try {
			const response = await axios.get(
				`http://localhost:${websocketPort}/api/${endpoint}`
			);
			updatePageWithData(endpoint, response.data);
		} catch (error) {
			console.error(`Error fetching ${endpoint} data:`, error);
		}
	}

	function pollData(timestamp) {
		Object.keys(intervals).forEach((endpoint) => {
			if (
				!lastTimestamps[endpoint] ||
				timestamp - lastTimestamps[endpoint] >= intervals[endpoint]
			) {
				fetchData(endpoint);
				lastTimestamps[endpoint] = timestamp;
			}
		});
		requestAnimationFrame(pollData);
	}

	ipcRenderer.on("config", (event, config) => {
		websocketPort = config.websocketPort;
		requestAnimationFrame(pollData);
	});

	const updateButton = document.getElementById("updateButton");
	const statusText = document.getElementById("status");

	updateButton.addEventListener("click", async () => {
		try {
			const response = await axios.get(
				`http://localhost:${websocketPort}/api/status`
			);
			statusText.textContent = `Status: ${response.data.status}`;
		} catch (error) {
			console.error("Error fetching status:", error);
			statusText.textContent = "Status: Error";
		}
	});
});
