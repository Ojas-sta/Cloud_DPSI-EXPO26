import os

repo_dir = "/Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent"
three_path = os.path.join(repo_dir, "assets", "three.min.js")
artifact_dir = "/Users/roopalisingh/.gemini/antigravity/brain/ab9c06fe-2bf2-446d-becc-2e4354fd5ec7"

with open(three_path, "r", encoding="utf-8") as f:
    three_js_code = f.read()

html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Project Cloud: 3D RViz & Gazebo Quadruped Simulation</title>
  <script src="https://www.gstatic.com/antigravity/web/dev/tailwindcss.min.js"></script>
  <script>
{three_js_code}
  </script>
  <style>
    body, html {{ margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #070a13; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }}
    #viewport {{ width: 100%; height: 100%; display: block; }}
    .hud-card {{ background: rgba(11, 16, 29, 0.88); backdrop-filter: blur(12px); border: 1px solid rgba(255, 255, 255, 0.1); }}
    .pip-cam {{ width: 220px; height: 140px; border: 2px solid #38bdf8; border-radius: 8px; overflow: hidden; position: relative; }}
  </style>
</head>
<body class="text-slate-100 antialiased select-none">

  <!-- 3D WebGL Canvas -->
  <div id="viewport" class="w-full h-full relative"></div>

  <!-- Top Navigation & Control Bar -->
  <div class="absolute top-4 left-4 right-4 flex flex-col md:flex-row items-start md:items-center justify-between gap-3 pointer-events-none">
    
    <!-- Title Badge -->
    <div class="hud-card px-4 py-2.5 rounded-xl pointer-events-auto flex items-center gap-3">
      <div class="w-3 h-3 rounded-full bg-emerald-400 animate-ping"></div>
      <div>
        <div class="text-[10px] font-mono tracking-widest text-emerald-400 font-bold uppercase">RViz / Gazebo 3D Digital Twin</div>
        <div class="text-sm font-bold text-white flex items-center gap-2">
          Project Cloud // Unitree Go2 Edu
          <span id="fsm-badge" class="px-2 py-0.5 rounded-full text-[10px] font-mono bg-indigo-500/30 text-indigo-300 border border-indigo-400/40">STATE_IDLE_STANDBY</span>
        </div>
      </div>
    </div>

    <!-- Camera Mode & Layer Toggles -->
    <div class="hud-card p-1.5 rounded-xl pointer-events-auto flex flex-wrap items-center gap-1.5 text-xs font-medium">
      <span class="text-[11px] font-mono text-slate-400 px-2">CAM:</span>
      <button onclick="setCameraMode('orbit')" id="btn-cam-orbit" class="px-2.5 py-1.5 rounded-lg bg-indigo-600 text-white font-semibold transition-colors">3D Orbit</button>
      <button onclick="setCameraMode('fpv')" id="btn-cam-fpv" class="px-2.5 py-1.5 rounded-lg bg-slate-800 text-slate-300 hover:bg-slate-700 transition-colors">Robot FPV (D435i)</button>
      <button onclick="setCameraMode('follow')" id="btn-cam-follow" class="px-2.5 py-1.5 rounded-lg bg-slate-800 text-slate-300 hover:bg-slate-700 transition-colors">Chase Follow</button>
      <button onclick="setCameraMode('top')" id="btn-cam-top" class="px-2.5 py-1.5 rounded-lg bg-slate-800 text-slate-300 hover:bg-slate-700 transition-colors">Top-Down Grid</button>

      <span class="text-slate-600 mx-1">|</span>
      <button onclick="toggleLayer('lidar')" id="btn-toggle-lidar" class="px-2 py-1.5 rounded-lg bg-emerald-950/80 text-emerald-400 border border-emerald-700/60 font-mono text-[11px]">LiDAR [ON]</button>
      <button onclick="toggleLayer('frustum')" id="btn-toggle-frustum" class="px-2 py-1.5 rounded-lg bg-sky-950/80 text-sky-400 border border-sky-700/60 font-mono text-[11px]">FOV [ON]</button>
      <button onclick="toggleLayer('tf')" id="btn-toggle-tf" class="px-2 py-1.5 rounded-lg bg-purple-950/80 text-purple-400 border border-purple-700/60 font-mono text-[11px]">TF Axes [ON]</button>
    </div>
  </div>

  <!-- Bottom Scene Timeline Controller -->
  <div class="absolute bottom-4 left-4 right-4 flex flex-col md:flex-row items-end md:items-center justify-between gap-3 pointer-events-none">

    <!-- Mission Scene Selector -->
    <div class="hud-card p-2 rounded-2xl pointer-events-auto flex items-center gap-1.5 overflow-x-auto max-w-full">
      <button onclick="triggerAutoPlay()" id="btn-auto-play" class="px-3.5 py-2 bg-indigo-600 hover:bg-indigo-500 text-white rounded-xl text-xs font-bold flex items-center gap-1.5 mr-1 shadow-lg shadow-indigo-600/30">
        <span id="play-symbol">▶</span> Auto-Play Tour
      </button>
      <button onclick="jumpToScene(0)" class="scene-btn px-3 py-1.5 rounded-xl text-left bg-indigo-950/60 border border-indigo-500/50 text-xs" data-s="0">
        <div class="text-[9px] font-mono text-indigo-400">01</div>
        <div class="font-bold text-white text-[11px]">VIP Greeting</div>
      </button>
      <button onclick="jumpToScene(1)" class="scene-btn px-3 py-1.5 rounded-xl text-left bg-slate-900/80 border border-slate-800 text-xs text-slate-300" data-s="1">
        <div class="text-[9px] font-mono text-slate-500">02</div>
        <div class="font-bold text-white text-[11px]">ARGUS Search</div>
      </button>
      <button onclick="jumpToScene(2)" class="scene-btn px-3 py-1.5 rounded-xl text-left bg-slate-900/80 border border-slate-800 text-xs text-slate-300" data-s="2">
        <div class="text-[9px] font-mono text-rose-500">03</div>
        <div class="font-bold text-white text-[11px]">Spill E-Stop</div>
      </button>
      <button onclick="jumpToScene(3)" class="scene-btn px-3 py-1.5 rounded-xl text-left bg-slate-900/80 border border-slate-800 text-xs text-slate-300" data-s="3">
        <div class="text-[9px] font-mono text-sky-400">04</div>
        <div class="font-bold text-white text-[11px]">ArUco Standoff</div>
      </button>
      <button onclick="jumpToScene(4)" class="scene-btn px-3 py-1.5 rounded-xl text-left bg-slate-900/80 border border-slate-800 text-xs text-slate-300" data-s="4">
        <div class="text-[9px] font-mono text-purple-400">05</div>
        <div class="font-bold text-white text-[11px]">180° Pivot</div>
      </button>
      <button onclick="jumpToScene(5)" class="scene-btn px-3 py-1.5 rounded-xl text-left bg-slate-900/80 border border-slate-800 text-xs text-slate-300" data-s="5">
        <div class="text-[9px] font-mono text-slate-400">06</div>
        <div class="font-bold text-white text-[11px]">Stage & Prone</div>
      </button>
    </div>

    <!-- Telemetry & Speech Subtitles Card -->
    <div class="hud-card p-3 rounded-2xl pointer-events-auto max-w-md w-full text-xs space-y-2">
      <div class="flex items-center justify-between font-mono text-[11px] border-b border-slate-800 pb-1.5">
        <span class="text-slate-400">TELEMETRY:</span>
        <span id="tel-pos" class="text-sky-300">X: 0.00m | Y: 0.00m</span>
        <span id="tel-yaw" class="text-indigo-300">Yaw: 0.0°</span>
        <span id="tel-bat" class="text-emerald-400 font-bold">98.0%</span>
      </div>
      <div class="flex items-start gap-2">
        <div class="w-2 h-2 rounded-full bg-indigo-400 mt-1 animate-pulse"></div>
        <p id="tts-subtitle" class="text-indigo-100 italic text-[11px] leading-relaxed">
          "Honorable evaluators, welcome to DPSI EXPO 2026. I am Cloud, your autonomous quadruped VIP docent..."
        </p>
      </div>
    </div>

  </div>

  <!-- RealSense PIP Camera Feed Overlay (Bottom Right) -->
  <div id="pip-container" class="absolute top-20 right-4 hud-card p-2 rounded-xl pointer-events-auto hidden md:block">
    <div class="text-[10px] font-mono text-sky-400 font-semibold mb-1 flex items-center justify-between">
      <span>REALSENSE D435i OPTICAL STREAM</span>
      <span class="w-2 h-2 rounded-full bg-emerald-400 animate-ping"></span>
    </div>
    <div class="pip-cam bg-black relative">
      <canvas id="pipCanvas" class="w-full h-full block"></canvas>
      <!-- AR Tag Bounding Box Overlay -->
      <div id="pip-overlay-aruco" class="absolute border-2 border-emerald-400 bg-emerald-500/20 text-emerald-300 font-mono text-[9px] p-0.5 pointer-events-none hidden" style="top:25px; left:65px; width:80px; height:80px;">
        ARUCO ID 10<br/>DIST: 1.80m
      </div>
      <!-- Spill Hazard Overlay -->
      <div id="pip-overlay-spill" class="absolute border-2 border-rose-500 bg-rose-500/30 text-rose-300 font-mono text-[9px] p-0.5 pointer-events-none hidden" style="bottom:10px; left:40px; width:130px; height:50px; border-radius: 50%;">
        HAZARD: SPILL 0.17m²
      </div>
    </div>
  </div>

  <!-- 3D Simulator Implementation -->
  <script>
    // -------------------------------------------------------------
    // Three.js 3D Scene Initialization
    // -------------------------------------------------------------
    const container = document.getElementById("viewport");
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x060913);
    scene.fog = new THREE.FogExp2(0x060913, 0.025);

    const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.position.set(-5, 6, 8);

    const renderer = new THREE.WebGLRenderer({{ antialias: true, alpha: false }});
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    // Ambient & Directional Lighting (Pavilion Sun & Accents)
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const sunLight = new THREE.DirectionalLight(0xffffff, 1.2);
    sunLight.position.set(10, 18, 12);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 0.5;
    sunLight.shadow.camera.far = 40;
    sunLight.shadow.camera.left = -15;
    sunLight.shadow.camera.right = 15;
    sunLight.shadow.camera.top = 15;
    sunLight.shadow.camera.bottom = -15;
    scene.add(sunLight);

    // Coordinate Floor Grid (RViz & Gazebo Style)
    const gridHelper = new THREE.GridHelper(30, 30, 0x38bdf8, 0x1e293b);
    gridHelper.position.y = 0.0;
    scene.add(gridHelper);

    // Axis Helper (X=Red, Y=Green, Z=Blue)
    const axisGroup = new THREE.Group();
    const worldAxes = new THREE.AxesHelper(1.5);
    axisGroup.add(worldAxes);
    scene.add(axisGroup);

    // Floor Runway Surface
    const floorGeo = new THREE.PlaneGeometry(30, 30);
    const floorMat = new THREE.MeshStandardMaterial({{ color: 0x0a0f1d, roughness: 0.85, metalness: 0.1 }});
    const floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.rotation.x = -Math.PI / 2;
    floorMesh.receiveShadow = true;
    scene.add(floorMesh);

    // -------------------------------------------------------------
    // Build 3D Pavilion Environment (Booths, ArUco Poles, Spill Hazard)
    // -------------------------------------------------------------
    const environmentGroup = new THREE.Group();

    // Central Pathway Corridor
    const pathGeo = new THREE.PlaneGeometry(3.0, 18.0);
    const pathMat = new THREE.MeshStandardMaterial({{ color: 0x141d33, roughness: 0.9 }});
    const pathMesh = new THREE.Mesh(pathGeo, pathMat);
    pathMesh.rotation.x = -Math.PI / 2;
    pathMesh.position.set(6.0, 0.005, 0.0);
    environmentGroup.add(pathMesh);

    // Booth 1: Autonomous Solar Tracking Cart (NODE_10 at X=12, Z=0)
    const b1Group = new THREE.Group();
    b1Group.position.set(12.0, 0, 0);

    // Table
    const tableMat = new THREE.MeshStandardMaterial({{ color: 0x1e293b, metalness: 0.3, roughness: 0.6 }});
    const tableTop = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.1, 2.4), tableMat);
    tableTop.position.set(0, 0.85, 0);
    tableTop.castShadow = true;
    b1Group.add(tableTop);

    // 3D Miniature Solar Tracker Cart on Table
    const cartBase = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.08, 0.4), new THREE.MeshStandardMaterial({{ color: 0x0284c7 }}));
    cartBase.position.set(0, 0.94, 0);
    b1Group.add(cartBase);

    // Solar Panel on Dual-Axis Gimbal
    const panelGeo = new THREE.BoxGeometry(0.7, 0.02, 0.5);
    const panelMat = new THREE.MeshStandardMaterial({{ color: 0x0f172a, metalness: 0.9, roughness: 0.2 }});
    const solarPanel = new THREE.Mesh(panelGeo, panelMat);
    solarPanel.position.set(0, 1.15, 0);
    solarPanel.rotation.z = 0.35; // Sun tracking tilt
    b1Group.add(solarPanel);

    // ArUco Marker ID 10 on Upright Pole
    const poleGeo = new THREE.CylinderGeometry(0.025, 0.025, 1.4);
    const pole = new THREE.Mesh(poleGeo, new THREE.MeshStandardMaterial({{ color: 0x64748b }}));
    pole.position.set(-0.8, 0.7, 0);
    b1Group.add(pole);

    const markerPlate = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.32, 0.32), new THREE.MeshBasicMaterial({{ color: 0xffffff }}));
    markerPlate.position.set(-0.8, 1.3, 0);
    // Draw ArUco pattern on face
    const markerInner = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.24, 0.24), new THREE.MeshBasicMaterial({{ color: 0x000000 }}));
    markerInner.position.set(-0.8, 1.3, 0);
    b1Group.add(markerPlate);
    b1Group.add(markerInner);

    environmentGroup.add(b1Group);

    // Chemical Spill Puddle (at X=6.0, Z=0.0)
    const spillGeo = new THREE.CircleGeometry(0.48, 32);
    const spillMat = new THREE.MeshStandardMaterial({{
      color: 0x0284c7,
      roughness: 0.05,
      metalness: 0.9,
      transparent: true,
      opacity: 0.85
    }});
    const spillMesh = new THREE.Mesh(spillGeo, spillMat);
    spillMesh.rotation.x = -Math.PI / 2;
    spillMesh.position.set(6.0, 0.015, 0.0);
    environmentGroup.add(spillMesh);

    // Spill Caution Cones
    for (let angle = 0; angle < Math.PI * 2; angle += Math.PI * 2 / 3) {{
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.25, 16), new THREE.MeshStandardMaterial({{ color: 0xf97316 }}));
      cone.position.set(6.0 + 0.65 * Math.cos(angle), 0.125, 0.65 * Math.sin(angle));
      environmentGroup.add(cone);
    }}

    // Keepout Safety Barrier Fence (Red Translucent)
    const fenceMat = new THREE.MeshStandardMaterial({{ color: 0xe11d48, transparent: true, opacity: 0.25, roughness: 0.5 }});
    const fenceMesh = new THREE.Mesh(new THREE.BoxGeometry(4.0, 1.0, 0.05), fenceMat);
    fenceMesh.position.set(6.0, 0.5, -4.0);
    environmentGroup.add(fenceMesh);

    // Central Demonstration Stage (NODE_30 at X=0, Z=8.5)
    const stageMesh = new THREE.Mesh(new THREE.CylinderGeometry(2.2, 2.4, 0.15, 32), new THREE.MeshStandardMaterial({{ color: 0x1e1b4b, metalness: 0.5, roughness: 0.4 }}));
    stageMesh.position.set(0.0, 0.075, 8.5);
    environmentGroup.add(stageMesh);

    // Recharge Docking Bay (NODE_99 at X=-4.0, Z=-4.0)
    const dockPlate1 = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.02, 0.5), new THREE.MeshStandardMaterial({{ color: 0xeab308, metalness: 0.9 }}));
    dockPlate1.position.set(-4.0, 0.01, -3.85);
    const dockPlate2 = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.02, 0.5), new THREE.MeshStandardMaterial({{ color: 0xeab308, metalness: 0.9 }}));
    dockPlate2.position.set(-4.0, 0.01, -4.15);
    environmentGroup.add(dockPlate1);
    environmentGroup.add(dockPlate2);

    scene.add(environmentGroup);

    // -------------------------------------------------------------
    // Unitree Go2 Edu Quadruped 3D Articulated Model
    // -------------------------------------------------------------
    const robotRoot = new THREE.Group();
    robotRoot.position.set(0, 0.32, 0);

    // Chassis Torso Body
    const chassisGeo = new THREE.BoxGeometry(0.65, 0.18, 0.28);
    const chassisMat = new THREE.MeshStandardMaterial({{ color: 0x94a3b8, metalness: 0.65, roughness: 0.35 }});
    const chassisMesh = new THREE.Mesh(chassisGeo, chassisMat);
    chassisMesh.castShadow = true;
    robotRoot.add(chassisMesh);

    // Front Camera Snout (RealSense D435i)
    const snout = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.09, 0.22), new THREE.MeshStandardMaterial({{ color: 0x0f172a, roughness: 0.2 }}));
    snout.position.set(0.33, 0.02, 0);
    snout.castShadow = true;
    robotRoot.add(snout);

    // RealSense Optical Lenses
    const lensL = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.01), new THREE.MeshBasicMaterial({{ color: 0x38bdf8 }}));
    lensL.rotation.z = Math.PI / 2;
    lensL.position.set(0.37, 0.02, 0.05);
    const lensR = lensL.clone();
    lensR.position.set(0.37, 0.02, -0.05);
    robotRoot.add(lensL);
    robotRoot.add(lensR);

    // NeoPixel Halo Ring Light & Emitter
    const haloRing = new THREE.Mesh(
      new THREE.TorusGeometry(0.06, 0.012, 16, 32),
      new THREE.MeshBasicMaterial({{ color: 0x6e1ee6 }})
    );
    haloRing.rotation.y = Math.PI / 2;
    haloRing.position.set(0.375, 0.02, 0);
    robotRoot.add(haloRing);

    const haloLight = new THREE.PointLight(0x6e1ee6, 2.5, 3.5);
    haloLight.position.set(0.42, 0.02, 0);
    robotRoot.add(haloLight);

    // Top 4D LiDAR Puck (Rotating)
    const lidarPuck = new THREE.Mesh(
      new THREE.CylinderGeometry(0.05, 0.05, 0.05, 24),
      new THREE.MeshStandardMaterial({{ color: 0x1e293b, metalness: 0.8 }})
    );
    lidarPuck.position.set(0.12, 0.12, 0);
    robotRoot.add(lidarPuck);

    // LiDAR Laser Scanning Fan (Translucent)
    const lidarFanGeo = new THREE.ConeGeometry(4.0, 0.02, 32, 1, true, 0, Math.PI / 3);
    const lidarFanMat = new THREE.MeshBasicMaterial({{ color: 0x10b981, transparent: true, opacity: 0.18, side: THREE.DoubleSide }});
    const lidarFan = new THREE.Mesh(lidarFanGeo, lidarFanMat);
    lidarFan.rotation.x = Math.PI / 2;
    lidarFan.position.set(0.12, 0.12, 0);
    robotRoot.add(lidarFan);

    // Rear Tactile Console Button (Green Push-to-Proceed)
    const btnMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.015), new THREE.MeshBasicMaterial({{ color: 0x22c55e }}));
    btnMesh.position.set(-0.25, 0.10, 0);
    robotRoot.add(btnMesh);

    // 4 Articulated Legs
    const legs = [];
    const legPositions = [
      {{ name: 'FL', x:  0.22, z:  0.16 }},
      {{ name: 'FR', x:  0.22, z: -0.16 }},
      {{ name: 'RL', x: -0.22, z:  0.16 }},
      {{ name: 'RR', x: -0.22, z: -0.16 }}
    ];

    const legMat = new THREE.MeshStandardMaterial({{ color: 0x475569, metalness: 0.5, roughness: 0.5 }});
    const footMat = new THREE.MeshStandardMaterial({{ color: 0x0f172a, roughness: 0.9 }});

    legPositions.forEach((pos, idx) => {{
      const hipGroup = new THREE.Group();
      hipGroup.position.set(pos.x, -0.05, pos.z);

      // Thigh
      const thigh = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.18, 0.04), legMat);
      thigh.position.y = -0.09;
      thigh.castShadow = true;
      hipGroup.add(thigh);

      // Shin (Lower Leg)
      const shin = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.18, 0.03), legMat);
      shin.position.set(0, -0.18, 0);
      shin.castShadow = true;
      hipGroup.add(shin);

      // Spherical Rubber Foot Pad
      const foot = new THREE.Mesh(new THREE.SphereGeometry(0.025, 16, 16), footMat);
      foot.position.set(0, -0.27, 0);
      foot.castShadow = true;
      hipGroup.add(foot);

      robotRoot.add(hipGroup);
      legs.push({{ group: hipGroup, thigh, shin, foot, sign: (idx % 2 === 0 ? 1 : -1) }});
    }});

    // RealSense Camera Optical Frustum Cone
    const frustumGeo = new THREE.ConeGeometry(1.6, 2.4, 4, 1, true);
    const frustumMat = new THREE.MeshBasicMaterial({{ color: 0x38bdf8, transparent: true, opacity: 0.15, wireframe: true }});
    const frustumMesh = new THREE.Mesh(frustumGeo, frustumMat);
    frustumMesh.rotation.z = -Math.PI / 2;
    frustumMesh.position.set(1.4, 0, 0);
    robotRoot.add(frustumMesh);

    // Robot TF Coordinate Axes (RGB)
    const robotTF = new THREE.AxesHelper(0.4);
    robotTF.position.set(0, 0.15, 0);
    robotRoot.add(robotTF);

    scene.add(robotRoot);

    // Navigation Trajectory Ribbon
    const trailPoints = [];
    const trailGeo = new THREE.BufferGeometry();
    const trailMat = new THREE.LineBasicMaterial({{ color: 0x38bdf8, linewidth: 2 }});
    const trailLine = new THREE.Line(trailGeo, trailMat);
    scene.add(trailLine);

    // -------------------------------------------------------------
    // Mission Scenes & Animation Keyframes
    // -------------------------------------------------------------
    const SCENE_KEYFRAMES = [
      {{
        name: "Scene 1: VIP Reception & Welcome",
        fsm: "STATE_DOCENT_GREET",
        pos: {{ x: 0.0, z: 0.0 }},
        yaw: 0.0,
        height: 0.32,
        pitch: 0.0,
        haloColor: 0x6e1ee6,
        haloMode: "breathe",
        speech: "Honorable evaluators and distinguished guests, welcome to the DPSI EXPO 2026. I am Cloud, your autonomous quadruped VIP tour docent and ground utility steward. Please press my green console button to begin."
      }},
      {{
        name: "Scene 2: Makerspace Inventory Query (ARGUS)",
        fsm: "STATE_ARGUS_PATROL",
        pos: {{ x: 2.5, z: 0.0 }},
        yaw: 0.0,
        height: 0.32,
        pitch: 0.0,
        haloColor: 0x00b4d8,
        haloMode: "breathe",
        speech: "Query: ESP32 microcontroller matched in 8.3ms. Located in Cabinet 01, Drawer B, Shelf Tier 1, referenced under Anchor Tag 1. 19 units in stock."
      }},
      {{
        name: "Scene 3: Unannounced Chemical Spill & E-Stop",
        fsm: "STATE_HAZARD_ALERT",
        pos: {{ x: 5.2, z: 0.0 }},
        yaw: 0.0,
        height: 0.32,
        pitch: 0.0,
        haloColor: 0xef4444,
        haloMode: "strobe",
        speech: "Caution! Floor liquid hazard intercepted (Area: 0.174 m², Conf: 99%). Emergency freeze engaged! Discord incident webhook dispatched with photo evidence."
      }},
      {{
        name: "Scene 4: Precision ArUco Standoff Alignment",
        fsm: "STATE_DOCENT_CRUISE",
        pos: {{ x: 10.2, z: 0.0 }},
        yaw: 0.0,
        height: 0.32,
        pitch: 0.0,
        haloColor: 0x38bdf8,
        haloMode: "breathe",
        speech: "ArUco Marker ID 10 locked in camera optical frame via solvePnP. Canonical 1.80m standoff line achieved within ±2cm."
      }},
      {{
        name: "Scene 5: 180° Presenter Pivot & Narration",
        fsm: "STATE_DOCENT_NARRATE",
        pos: {{ x: 10.2, z: 0.0 }},
        yaw: Math.PI,
        height: 0.28,
        pitch: 8.0,
        haloColor: 0xa855f7,
        haloMode: "breathe",
        speech: "Honorable evaluators, welcome to Booth 1: The Autonomous Solar Tracking Cart. Notice the dual-axis worm gear drive 3D-printed in PETG-CF. Achieving +34.2% daily energy yield on an STM32F401 Black Pill."
      }},
      {{
        name: "Scene 6: Central Stage Finale & Prone Rest",
        fsm: "STATE_PRONE_REST",
        pos: {{ x: -4.0, z: -4.0 }},
        yaw: -0.75 * Math.PI,
        height: 0.14,
        pitch: 0.0,
        haloColor: 0x334155,
        haloMode: "off",
        speech: "Tour concluded. Cloud docking into autonomous prone recharge bay. Thank you for your dedication to student robotics."
      }}
    ];

    let currentScene = 0;
    let cameraMode = 'orbit'; // 'orbit', 'fpv', 'follow', 'top'
    let autoPlayInterval = null;

    // Smooth Interpolation State
    let targetPos = {{ x: 0, z: 0 }};
    let targetYaw = 0;
    let targetHeight = 0.32;
    let targetPitch = 0.0;
    let currentPos = {{ x: 0, z: 0 }};
    let currentYaw = 0;
    let currentHeight = 0.32;
    let currentPitch = 0.0;
    let isMoving = false;

    // -------------------------------------------------------------
    // Orbit Camera Mouse Controller
    // -------------------------------------------------------------
    let isMouseDown = false;
    let mouseStart = {{ x: 0, y: 0 }};
    let orbitAngles = {{ theta: 0.8, phi: 1.1, radius: 10.0 }};
    let orbitTarget = new THREE.Vector3(0, 0.5, 0);

    function updateOrbitCamera() {{
      orbitAngles.phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, orbitAngles.phi));
      camera.position.x = orbitTarget.x + orbitAngles.radius * Math.sin(orbitAngles.phi) * Math.sin(orbitAngles.theta);
      camera.position.y = orbitTarget.y + orbitAngles.radius * Math.cos(orbitAngles.phi);
      camera.position.z = orbitTarget.z + orbitAngles.radius * Math.sin(orbitAngles.phi) * Math.cos(orbitAngles.theta);
      camera.lookAt(orbitTarget);
    }}

    container.addEventListener('mousedown', (e) => {{
      isMouseDown = true;
      mouseStart = {{ x: e.clientX, y: e.clientY }};
    }});
    window.addEventListener('mouseup', () => {{ isMouseDown = false; }});
    window.addEventListener('mousemove', (e) => {{
      if (!isMouseDown || cameraMode !== 'orbit') return;
      const dx = e.clientX - mouseStart.x;
      const dy = e.clientY - mouseStart.y;
      orbitAngles.theta -= dx * 0.007;
      orbitAngles.phi -= dy * 0.007;
      mouseStart = {{ x: e.clientX, y: e.clientY }};
      updateOrbitCamera();
    }});
    container.addEventListener('wheel', (e) => {{
      if (cameraMode !== 'orbit') return;
      e.preventDefault();
      orbitAngles.radius = Math.max(2.5, Math.min(25.0, orbitAngles.radius + e.deltaY * 0.015));
      updateOrbitCamera();
    }}, {{ passive: false }});

    // -------------------------------------------------------------
    // Scene Transitions & Jumping
    // -------------------------------------------------------------
    function jumpToScene(idx) {{
      currentScene = idx;
      const kf = SCENE_KEYFRAMES[idx];

      targetPos = {{ x: kf.pos.x, z: kf.pos.z }};
      targetYaw = kf.yaw;
      targetHeight = kf.height;
      targetPitch = kf.pitch;
      isMoving = true;

      // Update UI
      document.querySelectorAll(".scene-btn").forEach(btn => {{
        const bIdx = parseInt(btn.getAttribute("data-s"));
        if (bIdx === idx) {{
          btn.className = "scene-btn px-3 py-1.5 rounded-xl text-left bg-indigo-950/80 border border-indigo-400 text-xs shadow-lg";
        }} else {{
          btn.className = "scene-btn px-3 py-1.5 rounded-xl text-left bg-slate-900/80 border border-slate-800 text-xs text-slate-300";
        }}
      }});

      document.getElementById("fsm-badge").innerText = kf.fsm;
      document.getElementById("tts-subtitle").innerText = `"${{kf.speech}}"`;

      // Update NeoPixel Halo
      haloRing.material.color.setHex(kf.haloColor);
      haloLight.color.setHex(kf.haloColor);
      haloLight.intensity = kf.haloMode === 'off' ? 0.0 : 2.5;

      // Update PIP Overlays
      document.getElementById("pip-overlay-aruco").style.display = idx === 3 ? "block" : "none";
      document.getElementById("pip-overlay-spill").style.display = idx === 2 ? "block" : "none";
    }}

    function triggerAutoPlay() {{
      if (autoPlayInterval) {{
        clearInterval(autoPlayInterval);
        autoPlayInterval = null;
        document.getElementById("play-symbol").innerText = "▶";
      }} else {{
        document.getElementById("play-symbol").innerText = "⏸";
        autoPlayInterval = setInterval(() => {{
          let next = (currentScene + 1) % SCENE_KEYFRAMES.length;
          jumpToScene(next);
        }}, 4500);
      }}
    }}

    // Camera Mode Switching
    function setCameraMode(mode) {{
      cameraMode = mode;
      ['orbit', 'fpv', 'follow', 'top'].forEach(m => {{
        const b = document.getElementById(`btn-cam-${{m}}`);
        if (m === mode) {{
          b.className = "px-2.5 py-1.5 rounded-lg bg-indigo-600 text-white font-semibold transition-colors";
        }} else {{
          b.className = "px-2.5 py-1.5 rounded-lg bg-slate-800 text-slate-300 hover:bg-slate-700 transition-colors";
        }}
      }});
      if (mode === 'orbit') {{
        orbitTarget.set(currentPos.x, currentHeight, currentPos.z);
        updateOrbitCamera();
      }}
    }}

    // Visual Layers Toggle
    let showLidar = true;
    let showFrustum = true;
    let showTF = true;

    function toggleLayer(layer) {{
      if (layer === 'lidar') {{
        showLidar = !showLidar;
        lidarFan.visible = showLidar;
        document.getElementById('btn-toggle-lidar').innerText = `LiDAR [${{showLidar ? 'ON' : 'OFF'}}]`;
      }} else if (layer === 'frustum') {{
        showFrustum = !showFrustum;
        frustumMesh.visible = showFrustum;
        document.getElementById('btn-toggle-frustum').innerText = `FOV [${{showFrustum ? 'ON' : 'OFF'}}]`;
      }} else if (layer === 'tf') {{
        showTF = !showTF;
        robotTF.visible = showTF;
        worldAxes.visible = showTF;
        document.getElementById('btn-toggle-tf').innerText = `TF Axes [${{showTF ? 'ON' : 'OFF'}}]`;
      }}
    }}

    // -------------------------------------------------------------
    // Main Render & Animation Loop (60 FPS)
    // -------------------------------------------------------------
    let clock = new THREE.Clock();

    function animate() {{
      requestAnimationFrame(animate);
      const delta = clock.getDelta();
      const elapsed = clock.getElapsedTime();

      // Smooth Position & Orientation Interpolation
      const lerpFactor = Math.min(1.0, delta * 3.5);
      currentPos.x += (targetPos.x - currentPos.x) * lerpFactor;
      currentPos.z += (targetPos.z - currentPos.z) * lerpFactor;
      currentHeight += (targetHeight - currentHeight) * lerpFactor;
      currentPitch += (targetPitch - currentPitch) * lerpFactor;

      // Handle Yaw wrapping
      let dYaw = targetYaw - currentYaw;
      while (dYaw < -Math.PI) dYaw += Math.PI * 2;
      while (dYaw > Math.PI) dYaw -= Math.PI * 2;
      currentYaw += dYaw * lerpFactor;

      // Update 3D Robot Transforms
      robotRoot.position.set(currentPos.x, currentHeight, currentPos.z);
      robotRoot.rotation.y = currentYaw;
      chassisMesh.rotation.z = THREE.MathUtils.degToRad(-currentPitch);

      // LiDAR Puck Continuous 360° Rotation
      lidarPuck.rotation.y += delta * 12.0;
      lidarFan.rotation.z += delta * 12.0;

      // Articulated Trotting Walking Gait
      const distToTarget = Math.hypot(targetPos.x - currentPos.x, targetPos.z - currentPos.z);
      const isWalking = distToTarget > 0.05 && currentHeight > 0.20;

      legs.forEach((leg, idx) => {{
        if (currentHeight < 0.20) {{
          // Prone folding stance
          leg.group.rotation.x = Math.PI / 3;
          leg.thigh.position.y = -0.04;
        }} else if (isWalking) {{
          const trot = Math.sin(elapsed * 10.0 + (leg.sign > 0 ? 0 : Math.PI));
          leg.group.rotation.z = trot * 0.35;
          leg.thigh.position.y = -0.09 + Math.abs(trot) * 0.04;
        }} else {{
          // Idle stance
          leg.group.rotation.z = 0;
          leg.thigh.position.y = -0.09;
        }}
      }});

      // Record Odometry Breadcrumb Trail
      if (trailPoints.length === 0 || Math.hypot(currentPos.x - trailPoints[trailPoints.length - 1].x, currentPos.z - trailPoints[trailPoints.length - 1].z) > 0.2) {{
        trailPoints.push(new THREE.Vector3(currentPos.x, 0.02, currentPos.z));
        if (trailPoints.length > 200) trailPoints.shift();
        trailGeo.setFromPoints(trailPoints);
      }}

      // Camera Modes
      if (cameraMode === 'fpv') {{
        // Robot Eye View (RealSense D435i)
        camera.position.set(currentPos.x, currentHeight + 0.06, currentPos.z);
        const forward = new THREE.Vector3(Math.cos(currentYaw), -0.05, -Math.sin(currentYaw));
        camera.lookAt(camera.position.clone().add(forward));
      }} else if (cameraMode === 'follow') {{
        // Chase Cam
        const back = new THREE.Vector3(-Math.cos(currentYaw) * 2.8, 1.4, Math.sin(currentYaw) * 2.8);
        camera.position.copy(robotRoot.position).add(back);
        camera.lookAt(robotRoot.position.clone().add(new THREE.Vector3(0, 0.2, 0)));
      }} else if (cameraMode === 'top') {{
        // Top-Down Orthogonal View
        camera.position.set(currentPos.x, 14.0, currentPos.z);
        camera.lookAt(currentPos.x, 0, currentPos.z);
      }} else if (cameraMode === 'orbit') {{
        orbitTarget.set(currentPos.x, 0.4, currentPos.z);
        updateOrbitCamera();
      }}

      // Update HUD Values
      document.getElementById("tel-pos").innerText = `X: ${{currentPos.x.toFixed(2)}}m | Z: ${{currentPos.z.toFixed(2)}}m`;
      document.getElementById("tel-yaw").innerText = `Yaw: ${{(THREE.MathUtils.radToDeg(currentYaw)).toFixed(1)}}°`;

      renderer.render(scene, camera);
    }}

    // Window Resize Handling
    window.addEventListener('resize', () => {{
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    }});

    // Initialize Simulation at Scene 0 or URL parameter
    const urlParams = new URLSearchParams(window.location.search);
    const initialScene = urlParams.has('scene') ? parseInt(urlParams.get('scene')) : 0;
    jumpToScene(initialScene);
    updateOrbitCamera();
    animate();
  </script>
</body>
</html>
"""

# Write to repo
repo_target = os.path.join(repo_dir, "simulation_3d_rviz.html")
with open(repo_target, "w", encoding="utf-8") as f:
    f.write(html_content)
print(f"Generated 3D simulator in repo: {repo_target} ({len(html_content):,} bytes)")

# Write to artifact directory
artifact_target = os.path.join(artifact_dir, "simulation_3d_rviz.html")
with open(artifact_target, "w", encoding="utf-8") as f:
    f.write(html_content)
print(f"Generated 3D simulator in artifact dir: {artifact_target} ({len(html_content):,} bytes)")
