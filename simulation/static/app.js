const motorFile = document.querySelector("#motorFile");
const motorUrl = document.querySelector("#motorUrl");
const fetchButton = document.querySelector("#fetchButton");
const motorSummary = document.querySelector("#motorSummary");
const runButton = document.querySelector("#runButton");
const statusBox = document.querySelector("#status");
const summaryRows = document.querySelector("#summaryRows");
const plot = document.querySelector("#plot");
const components = document.querySelector("#components");
const addComponent = document.querySelector("#addComponent");
const bodyFile = document.querySelector("#bodyFile");
const bodySummary = document.querySelector("#bodySummary");
const csrfToken = document.querySelector('meta[name="csrf-token"]').content;

let loadedMotor = null;
const defaultComponents = [
  ["Airframe tube", 0.42, 0.48],
  ["Nose cone", 0.08, 0.88],
  ["Electronics", 0.18, 0.42],
  ["Battery", 0.12, 0.30],
  ["TVC mount", 0.22, 0.08],
];

defaultComponents.forEach(([name, mass, position]) => addComponentRow(name, mass, position));

addComponent.addEventListener("click", () => addComponentRow("Component", 0.01, 0.5));

bodyFile.addEventListener("change", async () => {
  const file = bodyFile.files[0];
  if (!file) {
    bodySummary.textContent = "No body export loaded.";
    return;
  }
  if (file.size > 2_000_000) {
    bodySummary.textContent = "Body export is too large.";
    return;
  }
  bodySummary.textContent = "Importing body export...";
  try {
    const response = await fetch("/api/import-body", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-CSRF-Token": csrfToken,
      },
      body: JSON.stringify({
        fileName: file.name,
        fileContent: await fileToBase64(file),
      }),
    });
    const payload = await response.json();
    if (!response.ok || payload.error) {
      throw new Error(payload.error || "Import failed.");
    }
    components.replaceChildren();
    payload.components.forEach((component) => addComponentRow(component.name, component.mass, component.position));
    setSpec("dryMass", payload.dryMass);
    if (payload.length) {
      setSpec("length", payload.length);
    }
    if (payload.radius) {
      setSpec("radius", payload.radius);
    }
    const massNote = payload.estimatedMassCount ? ` ${payload.estimatedMassCount} masses estimated; edit them before sim.` : "";
    bodySummary.textContent = `${payload.components.length} components imported. Dry mass ${format(payload.dryMass)} kg. Dry CG ${format(payload.dryCg)} m.${massNote}`;
  } catch (error) {
    bodySummary.textContent = error.message;
  }
});

motorFile.addEventListener("change", async () => {
  const file = motorFile.files[0];
  if (!file) {
    loadedMotor = null;
    motorSummary.textContent = "No motor loaded.";
    return;
  }
  if (file.size > 1_000_000) {
    loadedMotor = null;
    motorSummary.textContent = "Motor file is too large.";
    return;
  }
  loadedMotor = {
    fileName: file.name,
    fileContent: await file.text(),
  };
  motorSummary.textContent = `${file.name} loaded. Run the simulation to validate the curve.`;
});

fetchButton.addEventListener("click", async () => {
  const url = motorUrl.value.trim();
  if (!url) {
    motorSummary.textContent = "Enter a thrustcurve.org motor URL.";
    return;
  }
  fetchButton.disabled = true;
  motorSummary.textContent = "Fetching motor curve...";
  try {
    const response = await fetch("/api/fetch-motor", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-CSRF-Token": csrfToken,
      },
      body: JSON.stringify({ url }),
    });
    const payload = await response.json();
    if (!response.ok || payload.error) {
      throw new Error(payload.error || "Fetch failed.");
    }
    loadedMotor = payload;
    motorSummary.textContent = `${payload.fileName} fetched. Run the simulation to validate the curve.`;
  } catch (error) {
    loadedMotor = null;
    motorSummary.textContent = error.message;
  } finally {
    fetchButton.disabled = false;
  }
});

runButton.addEventListener("click", async () => {
  if (!loadedMotor) {
    statusBox.textContent = "Upload a thrust curve first.";
    return;
  }
  runButton.disabled = true;
  statusBox.textContent = "Running RocketPy...";
  summaryRows.replaceChildren();
  try {
    const response = await fetch("/api/simulate", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-CSRF-Token": csrfToken,
      },
      body: JSON.stringify({
        ...loadedMotor,
        specs: readSpecs(),
      }),
    });
    const payload = await response.json();
    if (!response.ok || payload.error) {
      throw new Error(payload.error || "Simulation failed.");
    }
    renderResults(payload);
    statusBox.textContent = `Complete using ${payload.engine}.`;
  } catch (error) {
    statusBox.textContent = error.message;
    clearPlot();
  } finally {
    runButton.disabled = false;
  }
});

function readSpecs() {
  const specs = {};
  document.querySelectorAll("[data-spec]").forEach((input) => {
    if (input.tagName === "SELECT") {
      specs[input.dataset.spec] = input.value === "true" ? true : input.value === "false" ? false : input.value;
      return;
    }
    specs[input.dataset.spec] = Number(input.value);
  });
  specs.components = Array.from(document.querySelectorAll(".component-row")).map((row) => ({
    name: row.querySelector("[data-component-name]").value.slice(0, 80),
    mass: Number(row.querySelector("[data-component-mass]").value),
    position: Number(row.querySelector("[data-component-position]").value),
  }));
  return specs;
}

function renderResults(payload) {
  motorSummary.textContent = [
    payload.motor.name,
    `burn ${format(payload.motor.burnTimeS)} s`,
    `impulse ${format(payload.motor.totalImpulseNS)} N s`,
    `peak ${format(payload.motor.peakThrustN)} N`,
  ].join(" | ");
  [
    ["Apogee", `${format(payload.summary.apogeeM)} m`],
    ["Apogee time", `${format(payload.summary.apogeeTimeS)} s`],
    ["Dry CG", `${format(payload.summary.dryCgM)} m`],
    ["Launch CG", `${format(payload.summary.launchCgM)} m`],
    ["Finless CP", `${format(payload.summary.cpM)} m`],
    ["Static margin", `${format(payload.summary.staticMarginCal)} cal`],
    ["Burnout", `${format(payload.summary.burnoutTimeS)} s`],
    ["Burnout altitude", `${format(payload.summary.burnoutAltitudeM)} m`],
    ["Burnout speed", `${format(payload.summary.burnoutSpeedMS)} m/s`],
    ["Flight time", `${format(payload.summary.flightTimeS)} s`],
    ["Max speed", `${format(payload.summary.maxSpeedMS)} m/s`],
    ["Max gimbal", `${format(payload.summary.maxGimbalDeg)} deg`],
    ["Final X", `${format(payload.summary.finalX)} m`],
    ["Final Y", `${format(payload.summary.finalY)} m`],
  ].forEach(([label, value]) => {
    const row = document.createElement("tr");
    const labelCell = document.createElement("td");
    const valueCell = document.createElement("td");
    labelCell.textContent = label;
    valueCell.textContent = value;
    row.append(labelCell, valueCell);
    summaryRows.append(row);
  });
  drawPlot(payload.samples, payload.motor.points);
}

function addComponentRow(name, mass, position) {
  const row = document.createElement("div");
  row.className = "component-row";
  const nameInput = document.createElement("input");
  const massInput = document.createElement("input");
  const positionInput = document.createElement("input");
  const removeButton = document.createElement("button");
  nameInput.dataset.componentName = "";
  nameInput.maxLength = 80;
  nameInput.ariaLabel = "Component name";
  nameInput.value = name;
  massInput.dataset.componentMass = "";
  massInput.type = "number";
  massInput.min = "0.001";
  massInput.step = "0.001";
  massInput.ariaLabel = "Component mass";
  massInput.value = mass;
  positionInput.dataset.componentPosition = "";
  positionInput.type = "number";
  positionInput.step = "0.001";
  positionInput.ariaLabel = "Component CG from tail";
  positionInput.value = position;
  removeButton.type = "button";
  removeButton.ariaLabel = "Remove component";
  removeButton.textContent = "Remove";
  removeButton.addEventListener("click", () => row.remove());
  row.append(nameInput, massInput, positionInput, removeButton);
  components.append(row);
}

function fileToBase64(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.addEventListener("load", () => resolve(String(reader.result).split(",", 2)[1] || ""));
    reader.addEventListener("error", () => reject(new Error("Could not read file.")));
    reader.readAsDataURL(file);
  });
}

function setSpec(key, value) {
  const input = document.querySelector(`[data-spec="${key}"]`);
  if (input) {
    input.value = Number(value).toFixed(4).replace(/0+$/, "").replace(/\.$/, "");
  }
}

function drawPlot(samples, motorPoints) {
  const context = plot.getContext("2d");
  const width = plot.width;
  const height = plot.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#fffdfa";
  context.fillRect(0, 0, width, height);
  const pad = { left: 58, right: 28, top: 24, bottom: 44 };
  const chartWidth = width - pad.left - pad.right;
  const chartHeight = height - pad.top - pad.bottom;
  const maxTime = Math.max(...samples.map((sample) => sample.time), ...motorPoints.map((point) => point.time));
  const maxAltitude = Math.max(1, ...samples.map((sample) => sample.altitude));
  const maxThrust = Math.max(1, ...motorPoints.map((point) => point.thrust));

  context.strokeStyle = "#d4d0c6";
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(pad.left, pad.top);
  context.lineTo(pad.left, height - pad.bottom);
  context.lineTo(width - pad.right, height - pad.bottom);
  context.stroke();

  drawSeries(
    context,
    samples.map((sample) => ({ x: sample.time, y: sample.altitude })),
    maxTime,
    maxAltitude,
    pad,
    chartWidth,
    chartHeight,
    "#5b513f",
  );
  drawSeries(context, motorPoints.map((point) => ({ x: point.time, y: point.thrust })), maxTime, maxThrust, pad, chartWidth, chartHeight, "#9a6b32");

  context.fillStyle = "#3c3932";
  context.font = "13px system-ui, sans-serif";
  context.fillText("Altitude", pad.left + 8, pad.top + 16);
  context.fillStyle = "#9a6b32";
  context.fillText("Thrust", pad.left + 88, pad.top + 16);
  context.fillStyle = "#625e53";
  context.fillText(`${format(maxAltitude)} m`, 8, pad.top + 6);
  context.fillText(`${format(maxTime)} s`, width - pad.right - 36, height - 14);
}

function drawSeries(context, points, maxX, maxY, pad, chartWidth, chartHeight, color) {
  context.strokeStyle = color;
  context.lineWidth = 2;
  context.beginPath();
  points.forEach((point, index) => {
    const x = pad.left + (point.x / maxX) * chartWidth;
    const y = pad.top + chartHeight - (point.y / maxY) * chartHeight;
    if (index === 0) {
      context.moveTo(x, y);
      return;
    }
    context.lineTo(x, y);
  });
  context.stroke();
}

function clearPlot() {
  plot.getContext("2d").clearRect(0, 0, plot.width, plot.height);
}

function format(value) {
  return Number(value).toLocaleString(undefined, {
    maximumFractionDigits: Math.abs(value) >= 100 ? 1 : 3,
  });
}
