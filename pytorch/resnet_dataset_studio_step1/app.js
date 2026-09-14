"use strict";

const DEFAULT_CLASSES = ["class_A", "class_B", "class_C", "class_D"];
const RECENT_HASH_LIMIT = 50;

const els = {
  video: document.querySelector("#video"),
  previewShell: document.querySelector(".preview-shell"),
  cameraStatus: document.querySelector("#cameraStatus"),
  startCameraBtn: document.querySelector("#startCameraBtn"),
  switchCameraBtn: document.querySelector("#switchCameraBtn"),
  chooseFolderBtn: document.querySelector("#chooseFolderBtn"),
  folderName: document.querySelector("#folderName"),
  classList: document.querySelector("#classList"),
  targetInput: document.querySelector("#targetInput"),
  intervalInput: document.querySelector("#intervalInput"),
  blurInput: document.querySelector("#blurInput"),
  blurToggle: document.querySelector("#blurToggle"),
  duplicateToggle: document.querySelector("#duplicateToggle"),
  mirrorToggle: document.querySelector("#mirrorToggle"),
  holdCaptureBtn: document.querySelector("#holdCaptureBtn"),
  autoCaptureBtn: document.querySelector("#autoCaptureBtn"),
  singleCaptureBtn: document.querySelector("#singleCaptureBtn"),
  statusMessage: document.querySelector("#statusMessage"),
  sessionStats: document.querySelector("#sessionStats"),
  captureCanvas: document.querySelector("#captureCanvas"),
  qualityCanvas: document.querySelector("#qualityCanvas"),
};

const state = {
  stream: null,
  storageReady: false,
  selectedIndex: 0,
  facingMode: "user",
  captureMode: null,
  loopTimer: null,
  captureBusy: false,
  savedThisSession: 0,
  rejectedThisSession: 0,
  classes: DEFAULT_CLASSES.map(function (name) {
    return { name: name, count: 0, rejected: 0, hashes: [] };
  }),
};

function targetCount() {
  return Math.max(1, Number.parseInt(els.targetInput.value, 10) || 1000);
}

function safeClassName(name) {
  return /^[\p{L}\p{N}_-]{1,50}$/u.test(name);
}

function setStatus(message, kind) {
  els.statusMessage.textContent = message;
  els.statusMessage.style.color =
    kind === "error" ? "var(--red)" : kind === "success" ? "var(--mint)" : "";
}

function updateSessionStats() {
  els.sessionStats.textContent =
    "저장 " + state.savedThisSession + " · 제외 " + state.rejectedThisSession;
}

function renderClasses() {
  const target = targetCount();
  els.classList.replaceChildren();

  state.classes.forEach(function (item, index) {
    const row = document.createElement("label");
    row.className = "class-row" + (index === state.selectedIndex ? " is-selected" : "");

    const radio = document.createElement("input");
    radio.type = "radio";
    radio.name = "selectedClass";
    radio.checked = index === state.selectedIndex;
    radio.addEventListener("change", function () {
      stopCapture();
      state.selectedIndex = index;
      renderClasses();
      setStatus(item.name + " 촬영 모드");
    });

    const nameInput = document.createElement("input");
    nameInput.type = "text";
    nameInput.value = item.name;
    nameInput.setAttribute("aria-label", "클래스 " + (index + 1) + " 이름");
    nameInput.addEventListener("change", async function () {
      const nextName = nameInput.value.trim();
      const previousName = item.name;
      if (!safeClassName(nextName)) {
        nameInput.value = item.name;
        setStatus("클래스 이름은 한글·영문·숫자·_·-만 사용할 수 있습니다.", "error");
        return;
      }
      const isDuplicateName = state.classes.some(function (other, otherIndex) {
        return otherIndex !== index && other.name === nextName;
      });
      if (isDuplicateName) {
        nameInput.value = item.name;
        setStatus("클래스 이름은 서로 달라야 합니다.", "error");
        return;
      }
      stopCapture();
      if (state.storageReady && nextName !== previousName) {
        try {
          await apiRequest("/api/rename", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ old: previousName, new: nextName }),
          });
        } catch (error) {
          nameInput.value = previousName;
          setStatus("클래스 이름 변경 실패: " + error.message, "error");
          return;
        }
      }
      item.name = nextName;
      item.hashes = [];
      if (state.storageReady) {
        await prepareStorage();
      }
      renderClasses();
    });

    const count = document.createElement("span");
    count.className = "class-count";
    count.textContent =
      item.count.toLocaleString() + " / " + target.toLocaleString();

    const track = document.createElement("span");
    track.className = "progress-track";
    const fill = document.createElement("span");
    fill.className = "progress-fill";
    fill.style.width = Math.min(100, (item.count / target) * 100) + "%";
    track.append(fill);

    row.append(radio, nameInput, count, track);
    els.classList.append(row);
  });
}

async function startCamera() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    setStatus("이 브라우저는 웹캠 API를 지원하지 않습니다.", "error");
    return;
  }

  stopCameraTracks();
  try {
    state.stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: { ideal: state.facingMode },
        width: { ideal: 1280 },
        height: { ideal: 720 },
      },
      audio: false,
    });
    els.video.srcObject = state.stream;
    await els.video.play();
    els.previewShell.classList.add("camera-ready");
    els.cameraStatus.textContent = "카메라 준비됨";
    els.cameraStatus.className = "status-pill status-ready";
    els.startCameraBtn.textContent = "카메라 재시작";
    setStatus("중앙 ROI 안에 객체를 크게 배치하세요.", "success");
  } catch (error) {
    setStatus("카메라를 열 수 없습니다: " + error.message, "error");
  }
}

function stopCameraTracks() {
  if (!state.stream) {
    return;
  }
  state.stream.getTracks().forEach(function (track) {
    track.stop();
  });
  state.stream = null;
}

async function switchCamera() {
  state.facingMode = state.facingMode === "user" ? "environment" : "user";
  await startCamera();
}

function classNames() {
  return state.classes.map(function (item) {
    return item.name;
  });
}

async function apiRequest(url, options) {
  const response = await fetch(url, options);
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.error || "서버 요청 실패");
  }
  return payload;
}

function applyCounts(counts) {
  for (const item of state.classes) {
    item.count = Number(counts[item.name] || 0);
    item.hashes = [];
  }
  renderClasses();
}

async function prepareStorage() {
  const names = classNames();
  if (new Set(names).size !== 4 || names.some(function (name) { return !safeClassName(name); })) {
    setStatus("서로 다른 유효한 클래스 이름 4개가 필요합니다.", "error");
    return;
  }
  try {
    const payload = await apiRequest("/api/classes", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ classes: names }),
    });
    state.storageReady = true;
    els.folderName.textContent = "저장 위치: " + payload.output;
    els.chooseFolderBtn.textContent = "데이터 폴더 새로고침";
    applyCounts(payload.counts);
    await saveCollectionState();
    setStatus("기존 이미지 수를 불러왔습니다. 이어서 촬영할 수 있습니다.", "success");
  } catch (error) {
    setStatus("데이터 폴더 준비 실패: " + error.message, "error");
  }
}

async function loadConfig() {
  try {
    const payload = await apiRequest("/api/config");
    els.folderName.textContent = "저장 예정 위치: " + payload.output;
  } catch (error) {
    setStatus("로컬 저장 서버 연결 실패: " + error.message, "error");
  }
}

function ensureReady() {
  if (!state.stream || els.video.readyState < 2) {
    setStatus("먼저 카메라를 시작하세요.", "error");
    return false;
  }
  if (!state.storageReady) {
    setStatus("먼저 데이터 폴더 준비를 누르세요.", "error");
    return false;
  }

  const item = state.classes[state.selectedIndex];
  if (!safeClassName(item.name)) {
    setStatus("유효한 클래스 이름을 입력하세요.", "error");
    return false;
  }
  if (item.count >= targetCount()) {
    setStatus(item.name + "은 이미 목표 수량에 도달했습니다.", "success");
    selectNextIncompleteClass();
    return false;
  }
  return true;
}

function drawRoiToCanvas() {
  const videoWidth = els.video.videoWidth;
  const videoHeight = els.video.videoHeight;
  const roiSize = Math.floor(Math.min(videoWidth, videoHeight) * 0.7);
  if (roiSize < 64) {
    throw new Error("ROI가 64×64보다 작습니다.");
  }

  const sourceX = Math.floor((videoWidth - roiSize) / 2);
  const sourceY = Math.floor((videoHeight - roiSize) / 2);
  const context = els.captureCanvas.getContext("2d", { alpha: false });
  context.save();
  context.clearRect(0, 0, 224, 224);
  if (els.mirrorToggle.checked) {
    context.translate(224, 0);
    context.scale(-1, 1);
  }
  context.drawImage(
    els.video,
    sourceX,
    sourceY,
    roiSize,
    roiSize,
    0,
    0,
    224,
    224
  );
  context.restore();
}

function analyzeFrame() {
  const context = els.qualityCanvas.getContext("2d", {
    willReadFrequently: true,
  });
  context.drawImage(els.captureCanvas, 0, 0, 32, 32);
  const pixels = context.getImageData(0, 0, 32, 32).data;
  const gray = new Float32Array(32 * 32);

  for (let index = 0; index < gray.length; index += 1) {
    const offset = index * 4;
    gray[index] =
      pixels[offset] * 0.299 +
      pixels[offset + 1] * 0.587 +
      pixels[offset + 2] * 0.114;
  }

  const laplacians = [];
  for (let y = 1; y < 31; y += 1) {
    for (let x = 1; x < 31; x += 1) {
      const index = y * 32 + x;
      laplacians.push(
        gray[index - 1] +
          gray[index + 1] +
          gray[index - 32] +
          gray[index + 32] -
          4 * gray[index]
      );
    }
  }

  const mean =
    laplacians.reduce(function (sum, value) {
      return sum + value;
    }, 0) / laplacians.length;
  const blurScore =
    laplacians.reduce(function (sum, value) {
      return sum + Math.pow(value - mean, 2);
    }, 0) / laplacians.length;

  let hash = "";
  for (let y = 0; y < 8; y += 1) {
    for (let x = 0; x < 8; x += 1) {
      const left = gray[y * 32 + x * 4];
      const right = gray[y * 32 + x * 4 + 3];
      hash += left > right ? "1" : "0";
    }
  }
  return { blurScore: blurScore, hash: hash };
}

function hammingDistance(first, second) {
  let distance = 0;
  const length = Math.min(first.length, second.length);
  for (let index = 0; index < length; index += 1) {
    if (first[index] !== second[index]) {
      distance += 1;
    }
  }
  return distance + Math.abs(first.length - second.length);
}

function canvasBlob() {
  return new Promise(function (resolve, reject) {
    els.captureCanvas.toBlob(
      function (blob) {
        if (blob) {
          resolve(blob);
        } else {
          reject(new Error("JPEG 변환 실패"));
        }
      },
      "image/jpeg",
      0.92
    );
  });
}

async function captureFrame() {
  if (state.captureBusy || !ensureReady()) {
    return false;
  }

  state.captureBusy = true;
  const item = state.classes[state.selectedIndex];
  try {
    drawRoiToCanvas();
    const quality = analyzeFrame();
    const blurLimit = Number.parseFloat(els.blurInput.value) || 0;

    if (els.blurToggle.checked && quality.blurScore < blurLimit) {
      item.rejected += 1;
      state.rejectedThisSession += 1;
      setStatus(
        "흐림 제외 · 선명도 " +
          quality.blurScore.toFixed(1) +
          " < " +
          blurLimit,
        "error"
      );
      updateSessionStats();
      return false;
    }

    const isDuplicate =
      els.duplicateToggle.checked &&
      item.hashes.some(function (hash) {
        return hammingDistance(hash, quality.hash) <= 2;
      });
    if (isDuplicate) {
      item.rejected += 1;
      state.rejectedThisSession += 1;
      setStatus(
        "최근 이미지와 너무 비슷합니다. 각도나 위치를 바꿔주세요.",
        "error"
      );
      updateSessionStats();
      return false;
    }

    const result = await apiRequest(
      "/api/capture?class=" +
        encodeURIComponent(item.name) +
        "&target=" +
        targetCount(),
      {
        method: "POST",
        headers: { "Content-Type": "image/jpeg" },
        body: await canvasBlob(),
      }
    );

    item.count = Number(result.count);
    item.hashes.push(quality.hash);
    if (item.hashes.length > RECENT_HASH_LIMIT) {
      item.hashes.shift();
    }
    state.savedThisSession += 1;
    renderClasses();
    updateSessionStats();
    setStatus(
      item.name +
        ": " +
        item.count +
        " / " +
        targetCount() +
        " 저장 · 선명도 " +
        quality.blurScore.toFixed(1),
      "success"
    );

    if (item.count % 25 === 0 || item.count >= targetCount()) {
      await saveCollectionState();
    }
    if (item.count >= targetCount()) {
      stopCapture();
      setStatus(
        item.name + " 목표 완료! 다음 미완료 클래스로 이동합니다.",
        "success"
      );
      selectNextIncompleteClass();
    }
    return true;
  } catch (error) {
    stopCapture();
    setStatus("촬영 실패: " + error.message, "error");
    return false;
  } finally {
    state.captureBusy = false;
  }
}

function selectNextIncompleteClass() {
  const target = targetCount();
  const nextIndex = state.classes.findIndex(function (item, index) {
    return index !== state.selectedIndex && item.count < target;
  });
  if (nextIndex >= 0) {
    state.selectedIndex = nextIndex;
  } else {
    setStatus("모든 클래스가 목표 수량에 도달했습니다.", "success");
  }
  renderClasses();
}

async function saveCollectionState() {
  if (!state.storageReady) {
    return;
  }
  await apiRequest("/api/metadata", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      target_per_class: targetCount(),
      classes: classNames(),
    }),
  });
}

function startCapture(mode) {
  if (state.captureMode || !ensureReady()) {
    return;
  }
  state.captureMode = mode;
  els.holdCaptureBtn.classList.toggle("is-recording", mode === "hold");
  els.autoCaptureBtn.classList.toggle("is-recording", mode === "auto");
  els.autoCaptureBtn.textContent =
    mode === "auto" ? "자동 촬영 중지" : "자동 촬영 시작";
  captureLoop();
}

async function captureLoop() {
  if (!state.captureMode) {
    return;
  }
  await captureFrame();
  if (!state.captureMode) {
    return;
  }
  const interval = Math.max(
    80,
    Number.parseInt(els.intervalInput.value, 10) || 150
  );
  state.loopTimer = window.setTimeout(captureLoop, interval);
}

function stopCapture() {
  state.captureMode = null;
  if (state.loopTimer) {
    window.clearTimeout(state.loopTimer);
  }
  state.loopTimer = null;
  els.holdCaptureBtn.classList.remove("is-recording");
  els.autoCaptureBtn.classList.remove("is-recording");
  els.autoCaptureBtn.textContent = "자동 촬영 시작";
}

els.startCameraBtn.addEventListener("click", startCamera);
els.switchCameraBtn.addEventListener("click", switchCamera);
els.chooseFolderBtn.addEventListener("click", prepareStorage);
els.singleCaptureBtn.addEventListener("click", captureFrame);
els.autoCaptureBtn.addEventListener("click", function () {
  if (state.captureMode === "auto") {
    stopCapture();
  } else {
    startCapture("auto");
  }
});
els.holdCaptureBtn.addEventListener("pointerdown", function (event) {
  event.preventDefault();
  els.holdCaptureBtn.setPointerCapture(event.pointerId);
  startCapture("hold");
});
["pointerup", "pointercancel", "pointerleave"].forEach(function (eventName) {
  els.holdCaptureBtn.addEventListener(eventName, stopCapture);
});
els.targetInput.addEventListener("change", renderClasses);
document.addEventListener("keydown", function (event) {
  const tagName = document.activeElement.tagName;
  if (
    event.code === "Space" &&
    tagName !== "INPUT" &&
    tagName !== "BUTTON"
  ) {
    event.preventDefault();
    captureFrame();
  }
});
window.addEventListener("beforeunload", function () {
  stopCapture();
  stopCameraTracks();
});

renderClasses();
updateSessionStats();
loadConfig();
