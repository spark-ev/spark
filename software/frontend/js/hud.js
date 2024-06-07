document.addEventListener("DOMContentLoaded", () => {
	const speedometer = document.getElementById("speedometer");
	let speed = 0;

	function updateSpeed() {
		speed = (Math.random() * 100).toFixed(2); // Simulate speed change
		speedometer.textContent = `Speed: ${speed} km/h`;
	}

	setInterval(updateSpeed, 1000); // Update speed every second
});
