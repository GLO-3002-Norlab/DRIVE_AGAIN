const socket = io();
let stage = 0;

function skipCommandButtonClick() {
  socket.emit("skip_command");
}

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

function saveDataset(event) {
  event.preventDefault();
  const datasetName = document.getElementById("dataset-name-input").value;
  socket.emit("save_dataset", { name: datasetName });
  console.log("Dataset saved with name:", datasetName);
}

socket.on("input_space_update", (data) => {
  document.getElementById("input_space").src =
    `data:image/png;base64,${data.image_data}`;
});

socket.on("robot_vizualisation_update", (data) => {
  document.getElementById("robot_viz").src =
    `data:image/png;base64,${data.image_data}`;
});

socket.on("skippable_state_start", (data) => {
  document.getElementById("skip-command-button").disabled = false;
});

socket.on("skippable_state_end", (data) => {
  document.getElementById("skip-command-button").disabled = true;
});


//
// Forms
//

document.getElementById("roboticist-select").addEventListener("change", handleSelectChange);
document.getElementById("robot-select").addEventListener("change", handleSelectChange);
document.getElementById("terrain-select").addEventListener("change", handleSelectChange);

function handleSelectChange(event) {
  const selectElement = event.target;
  const selectedValue = selectElement.value;

  if (selectedValue === "new") {
    openModal(selectElement.id);
    selectElement.selectedIndex = 0;
  }
}

function openModal(selectId) {
  sessionStorage.setItem("currentSelect", selectId);

  document.querySelectorAll(".modal-form").forEach(form => form.style.display = "none");
  document.getElementById(selectId + "-form").style.display = "block";
  document.getElementById("modal").style.display = "flex";
}

function closeModal() {
  document.getElementById("modal").style.display = "none";
}

document.querySelectorAll(".modal-form").forEach(form => {
  form.addEventListener("submit", function (event) {
    event.preventDefault();
    const currentSelect = sessionStorage.getItem("currentSelect");

    let newOptionValue;
    if (currentSelect === "roboticist-select") {
      const name = document.getElementById("newRoboticistName").value.trim();
      const lastName = document.getElementById("newRoboticistLastName").value.trim();
      const email = document.getElementById("newRoboticistEmail").value.trim();
      const experience = document.getElementById("newRoboticistExperience").value.trim();
      const org = document.getElementById("newRoboticistOrg").value.trim();

      if (!name || !lastName || !email || !experience || !org) return;
      newOptionValue = `${name} ${lastName} (${org})`;
    } else {
      newOptionValue = this.querySelector(".newOptionValue").value.trim();
    }

    if (newOptionValue) {
      const selectElement = document.getElementById(currentSelect);
      const newOption = document.createElement("option");
      newOption.value = newOptionValue.toLowerCase().replace(/\s+/g, "_");
      newOption.textContent = newOptionValue;
      const lastOption = selectElement.options[selectElement.options.length - 1];
      selectElement.insertBefore(newOption, lastOption);
      selectElement.value = newOption.value;
      closeModal();
    }
  });
});
