const { spawn } = require("child_process");
const path = require("path");
const fs = require("fs");

// Read ports.json
const ports = JSON.parse(
	fs.readFileSync(path.join(__dirname, "config", "ports.json"), "utf-8")
);
const websocketPort = ports.websocket;

// Read config.json
const config = JSON.parse(
	fs.readFileSync(path.join(__dirname, "config", "config.json"), "utf-8")
);

let flask = null;
let electron = null;
let ros = null;

// Function to start the Flask server
function startFlask() {
	flask = spawn("python3", ["main.py", websocketPort], {
		cwd: path.join(__dirname, "backend"),
	});

	flask.stdout.on("data", (data) => {
		if (config.debug) console.log(`Flask: ${data}`);
	});

	flask.stderr.on("data", (data) => {
		if (data.toString().includes(`HTTP/1.1" 200`)) return;
		if (config.debug) console.error(`Flask error: ${data}`);
	});

	flask.on("close", (code) => {
		if (config.debug) console.log(`Flask process exited with code ${code}`);
		cleanup();
	});
}

// Function to start the ROS node
function startROS() {
	ros = spawn("rosrun", ["my_car_package", "controller.py"], {
		cwd: path.join(__dirname, "backend"),
	});

	ros.stdout.on("data", (data) => {
		if (config.debug) console.log(`ROS: ${data}`);
	});

	ros.stderr.on("data", (data) => {
		if (config.debug) console.error(`ROS error: ${data}`);
	});

	ros.on("close", (code) => {
		if (config.debug) console.log(`ROS process exited with code ${code}`);
		cleanup();
	});
}

// Function to start the Electron app
function startElectron() {
	electron = spawn(
		"npx",
		["electron", "backend/main.js", websocketPort],
		{
			cwd: __dirname,
		}
	);

	electron.stdout.on("data", (data) => {
		if (config.debug) console.log(`Electron: ${data}`);
	});

	electron.stderr.on("data", (data) => {
		if (config.debug) console.error(`Electron error: ${data}`);
	});

	electron.on("close", (code) => {
		if (config.debug)
			console.log(`Electron process exited with code ${code}`);
		cleanup();
	});
}

function cleanup() {
	if (flask) flask.kill();
	if (ros) ros.kill();
	if (electron) electron.kill();
}

// Handle process exit events
process.on('exit', cleanup);
process.on('SIGINT', cleanup);
process.on('SIGTERM', cleanup);
process.on('uncaughtException', cleanup);

// Function to show intro message
function printLauncher() {
	console.log(`
  _________                   __
 /   _____/__________ _______|  | __
 \\_____  \\\\____ \\__  \\\\_  __ \\  |/ /
 /        \\  |_> > __ \\|  | \\/    <
/_______  /   __(____  /__|  |__|_ \\
        \\/|__|       \\/           \\/

v${config.version}
by Owen & Peter\n`);
}

// Start Flask, ROS, and Electron
printLauncher();
startFlask();
//startROS();
startElectron();
