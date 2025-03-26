function updateTractionFields() {
    const tractionType = document.getElementById("robotTraction").value;
    const tractionDetails = document.getElementById("traction-details");
    tractionDetails.innerHTML = "";

    if (tractionType === "wheeled") {
        tractionDetails.innerHTML = `
        <label for="tireModel">Tire model:</label>
        <input type="text" id="tireModel" required />
  
        <label for="treadDepth">Tread depth:</label>
        <input type="text" id="treadDepth" required />
  
        <label for="tirePressure">Tire pressure (starting from front-right, clockwise):</label>
        <input type="text" id="tirePressure" required />
      `;
    } else if (tractionType === "tracked") {
        tractionDetails.innerHTML = `
        <label for="trackModel">Model of tracks:</label>
        <input type="text" id="trackModel" required />
      `;
    } else if (tractionType === "legged") {
        tractionDetails.innerHTML = `
        <label for="numLegs">Number of legs:</label>
        <input type="number" id="numLegs" min="1" required />
      `;
    } else if (tractionType === "other") {
        tractionDetails.innerHTML = `
        <label for="otherTraction">Specify traction mechanism:</label>
        <input type="text" id="otherTraction" required />
      `;
    }
}

document.getElementById("robot-select-form").addEventListener("submit", function (event) {
    event.preventDefault();

    const name = document.getElementById("robotName").value.trim();
    const manufacturer = document.getElementById("robotManufacturer").value.trim();
    const modification = document.getElementById("robotModification").value;
    const weight = document.getElementById("robotWeight").value.trim();
    const asymmetry = document.getElementById("robotAsymmetry").value;
    const traction = document.getElementById("robotTraction").value;
    let tractionDetails = "";

    if (traction === "wheeled") {
        const tireModel = document.getElementById("tireModel").value.trim();
        const treadDepth = document.getElementById("treadDepth").value.trim();
        const tirePressure = document.getElementById("tirePressure").value.trim();
        tractionDetails = `Wheeled: ${tireModel}, ${treadDepth}, ${tirePressure}`;
    } else if (traction === "tracked") {
        tractionDetails = `Tracked: ${document.getElementById("trackModel").value.trim()}`;
    } else if (traction === "legged") {
        tractionDetails = `Legged: ${document.getElementById("numLegs").value} legs`;
    } else if (traction === "other") {
        tractionDetails = `Other: ${document.getElementById("otherTraction").value.trim()}`;
    }

    const suspension = document.getElementById("robotSuspension").value;
    const baseline = document.getElementById("robotBaseline").value.trim();
    const wheelRadius = document.getElementById("robotWheelRadius").value.trim();
    const sensors = document.getElementById("robotSensors").value.trim();
    const algorithm = document.getElementById("robotAlgorithm").value.trim();
    const speedTest = document.getElementById("robotSpeedTest").value;

    if (!manufacturer || !weight || !baseline || !wheelRadius || !sensors || !algorithm) {
        alert("Please fill out all required fields.");
        return;
    }

    const newRobotName = `${name} (${manufacturer})`;

    const selectElement = document.getElementById("robot-select");
    const newOption = document.createElement("option");
    newOption.value = newRobotName.toLowerCase().replace(/\s+/g, "_");
    newOption.textContent = newRobotName;

    const lastOption = selectElement.options[selectElement.options.length - 1];
    selectElement.insertBefore(newOption, lastOption);
    selectElement.value = newOption.value;

    closeModal();
});