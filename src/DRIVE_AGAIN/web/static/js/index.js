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

function saveDataset() {
  socket.emit("save_dataset");
}

socket.on("input_space_update", (data) => {
  document.getElementById("input_space").src =
    `data:image/png;base64,${data.image_data}`;
});

socket.on("robot_vizualisation_update", (data) => {
  document.getElementById("robot_viz").src =
    `data:image/png;base64,${data.image_data}`;
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
    const newOptionValue = this.querySelector(".newOptionValue").value.trim();
    const currentSelect = sessionStorage.getItem("currentSelect");

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
