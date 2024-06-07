document.addEventListener('keydown', function (event) {
	if (event.keyCode == 54) {
		if (document.getElementById("overlayTesla").style.display == "block") {
			document.getElementById("overlayTesla").style.display = "none";
		} else {
			document.getElementById("overlayTesla").style.display = "block";
		}
	}
});