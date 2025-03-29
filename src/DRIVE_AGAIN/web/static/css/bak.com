body {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin: 0;
  padding: 0;
  height: 100vh;
  max-width: 100%;
}

.top-bar {
  display: flex;
  justify-content: space-between;
  flex-wrap: wrap;
  width: 99%;
  position: fixed;
  top: 0;
}

.image-container {
  margin-top: 4rem; 
  display: flex;
  flex-grow: 1;
  flex-wrap: wrap;
  justify-content: center;
  align-items: center;
  width: 100%;
}

.select-container {
  display: flex;
  align-items: center;
  align-content: space-between;
}

.bottom-bar {
  display: flex;
  justify-content: flex-start;
  width: 99%;
  position: fixed;
  bottom: 0;
}

.save-dataset-form {
  display: flex;
  align-items: center;
  padding: 0.5rem;
}

.save-dataset-form label {
  margin-right: 1rem;
}

.image-container img {
  flex: 1;
  max-width: 100%;
  max-height: 100%;
}

@media (max-width: 600px) {
  .top-bar {
    flex-direction: column;
    align-items: stretch;
    padding: 1rem;
  }

  .select-container {
    flex-direction: column;
    align-items: stretch;
    gap: 0.5rem;
  }

  .save-dataset-form {
    flex-direction: column;
    align-items: stretch;
    margin-top: 1rem;
    gap: 0.5rem;
  }

  .save-dataset-form label {
    margin-right: 0;
    margin-bottom: 0.25rem;
  }
}

#mainButton {
  width: 100%;
  max-width: 100%;
}


<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />

    <title>DRIVE AGAIN</title>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.5.4/socket.io.js"></script>
    <link
      rel="stylesheet"
      href="https://cdn.jsdelivr.net/npm/water.css@2/out/water.css"
    />
    <link rel="stylesheet" href="/static/css/index.css" />
  </head>

  <body>
    <div class="top-bar">
      <div class="select-container">
        <select id="roboticist-select">
          <option value="">Select Roboticist</option>
          <option value="roboticist1">Roboticist 1</option>
          <option value="roboticist2">Roboticist 2</option>
          <option value="new">+ Create New</option>
        </select>

        <select id="robot-select">
          <option value="">Select Robot</option>
          <option value="robot1">Robot 1</option>
          <option value="robot2">Robot 2</option>
          <option value="new">+ Create New</option>
        </select>

        <select id="terrain-select">
          <option value="">Select Terrain</option>
          <option value="terrain1">Terrain 1</option>
          <option value="terrain2">Terrain 2</option>
          <option value="new">+ Create New</option>
        </select>
      </div>
      <form
        class="save-dataset-form"
        action="/submit"
        onsubmit="saveDataset(event)"
      >
        <label for="dataset-name-input">Dataset name : </label>
        <input
          type="text"
          id="dataset-name-input"
          name="datasetName"
          required
        />
        <button type="submit">Save dataset</button>
      </form>
    </div>

    <main class="image-container">
      <img id="robot_viz" />
      <img id="input_space" />
    </main>

    <div class="bottom-bar">
      <button id="mainButton" onclick="handleButtonClick()">
        Start Geofencing
      </button>
    </div>

    {% include "modal.html" %}

    <script defer="defer" src="/static/js/index.js"></script>
  </body>
</html>