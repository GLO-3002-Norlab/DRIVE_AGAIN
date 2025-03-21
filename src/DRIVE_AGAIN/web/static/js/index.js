const socket = io();
let stage = 0;

function handleButtonClick() {
  const button = document.getElementById("mainButton");

  if (stage === 0) {
    socket.emit("start_geofencing");
    button.textContent = "Stop Geofencing";
  } else if (stage === 1) {
    socket.emit("start_drive");
    button.textContent = "Drive Started";
    button.disabled = true;
  } 
  stage++;
}

socket.on("input_space_update", (data) => {
  document.getElementById(
    "input_space"
  ).src = `data:image/png;base64,${data.image_data}`;
});

socket.on("robot_vizualisation_update", (data) => {
  document.getElementById(
    "robot_viz"
  ).src = `data:image/png;base64,${data.image_data}`;
});
