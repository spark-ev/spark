const { app, BrowserWindow } = require("electron");
const path = require("path");

const websocketPort = process.argv[2];

let mainWindow;
let hudWindow;

function createWindow() {
	// Create the main window
	mainWindow = new BrowserWindow({
		width: 800,
		height: 600,
		webPreferences: {
			preload: path.join(__dirname, "../frontend/js/home.js"),
			nodeIntegration: true,
			contextIsolation: false,
		},
	});

	mainWindow.loadFile(path.join(__dirname, "../frontend/html/home.html"));
	mainWindow.webContents.on("did-finish-load", () => {
		mainWindow.webContents.send("config", { websocketPort });
	});

	// Create the HUD window
	hudWindow = new BrowserWindow({
		width: 400,
		height: 300,
		webPreferences: {
			preload: path.join(__dirname, "../frontend/js/hud.js"),
			nodeIntegration: true,
			contextIsolation: false,
		},
	});

	hudWindow.loadFile(path.join(__dirname, "../frontend/html/hud.html"));

	// Handle window close events
	mainWindow.on("closed", () => {
		mainWindow = null;
		if (hudWindow) {
			hudWindow.close(); // Close the HUD window if the main window is closed
		}
		app.quit();
	});

	hudWindow.on("closed", () => {
		hudWindow = null;
		if (mainWindow) {
			mainWindow.close(); // Close the main window if the HUD window is closed
		}
		app.quit();
	});
}

app.whenReady().then(createWindow);

app.on("window-all-closed", () => {
	if (process.platform !== "darwin") {
		app.quit();
	}
});

app.on("activate", () => {
	if (BrowserWindow.getAllWindows().length === 0) {
		createWindow();
	}
});
