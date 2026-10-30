/* FAFU 6-DoF reference arm. Joint origins match fafu_baseV1.urdf
   (same chain as fafu_follower.urdf). Visuals load SolidWorks STL
   (downsampled /static/meshes/*.bin). Fallback: cylinders/boxes. */
(function (global) {
  "use strict";

  var JOINTS = [
    { xyz: [0, 0, 0.0584], axis: [0, 0, 1], r: 0.028, color: 0x8b9aab },
    { xyz: [0.018199, 0, 0.053], axis: [0, 1, 0], r: 0.022, color: 0x5ec8ff },
    { xyz: [-0.26, 0, 0], axis: [0, -1, 0], r: 0.020, color: 0x9aa8b8 },
    { xyz: [0.23, 0, 0.06], axis: [0, -1, 0], r: 0.018, color: 0x5ec8ff },
    { xyz: [0.07, 0, 0.036319], axis: [0, 0, -1], r: 0.014, color: 0x9aa8b8 },
    { xyz: [0.02345, 0, -0.039], axis: [1, 0, 0], r: 0.012, color: 0x5ec8ff },
  ];
  var MESH_NAMES = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6"];
  /* First CAD palette: segmented light greys (not URDF 0.75 wash, not black joints). */
  var MESH_COLORS = [0x8a9199, 0x9aa3ad, 0xb8c0c8, 0x9aa3ad, 0xb8c0c8, 0x9aa3ad, 0xc5cdd4];
  var TOOL = [0.165, 0, 0];
  // One context and one set of immutable CAD buffers for every workspace.
  // Views own only their transforms, camera, gripper state and path line.
  var sharedRenderer = null;
  var activeView = null;
  var meshPromise = null;
  var cadMaterials = Object.create(null);
  var renderWidth = 0, renderHeight = 0, renderDpr = 0;
  var FRAME_MS = 1000 / 30;

  function themeColor(name, fallback) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
  }

  function getRenderer() {
    if (!sharedRenderer) {
      sharedRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
      sharedRenderer.setClearColor(themeColor("--view-bg", 0x070b10), 1);
      sharedRenderer.domElement.style.display = "block";
      sharedRenderer.domElement.style.width = "100%";
      sharedRenderer.domElement.style.height = "100%";
    }
    return sharedRenderer;
  }

  function cadMaterial(color) {
    if (!cadMaterials[color]) cadMaterials[color] = new THREE.MeshPhongMaterial({
      color: color,
      specular: 0x2a3038,
      shininess: 48,
      side: THREE.DoubleSide,
      flatShading: true,
    });
    return cadMaterials[color];
  }

  function geoFromTris(tris) {
    if (!tris || tris.length < 9) return null;
    var geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(tris), 3));
    geo.computeVertexNormals();
    geo.computeBoundingSphere();
    return geo;
  }

  function splitLink6(geo) {
    if (!geo) return null;
    var attr = geo.getAttribute("position");
    if (!attr || !attr.array) return null;
    var pos = attr.array;
    var body = [];
    var left = [];
    var right = [];
    var tipInner = 1e9;
    var i;
    for (i = 0; i + 8 < pos.length; i += 9) {
      var y0 = pos[i + 1], y1 = pos[i + 4], y2 = pos[i + 7];
      var x0 = pos[i], x1 = pos[i + 3], x2 = pos[i + 6];
      var maxX = Math.max(x0, x1, x2);
      var dest = body;
      if (maxX > 0.04 && y0 > 0.016 && y1 > 0.016 && y2 > 0.016) dest = left;
      else if (maxX > 0.04 && y0 < -0.016 && y1 < -0.016 && y2 < -0.016) dest = right;
      dest.push(x0, y0, pos[i + 2], x1, y1, pos[i + 5], x2, y2, pos[i + 8]);
      if (dest === left && maxX > 0.10) {
        tipInner = Math.min(tipInner, y0, y1, y2);
      }
    }
    if (left.length < 90 || right.length < 90) return null;
    var travel = 0.042;
    if (tipInner < 1e8) travel = Math.max(0.028, Math.min(0.07, tipInner - 0.003));
    return {
      body: geoFromTris(body) || geo,
      left: geoFromTris(left),
      right: geoFromTris(right),
      travel: travel,
    };
  }

  function parseFarm(buf) {
    if (!buf || buf.byteLength < 12) return null;
    var v = new DataView(buf);
    if (v.getUint8(0) !== 0x46 || v.getUint8(1) !== 0x41 || v.getUint8(2) !== 0x52 || v.getUint8(3) !== 0x4d) {
      return null;
    }
    var n = v.getUint32(8, true);
    if (n < 1 || buf.byteLength < 12 + n * 36) return null;
    var src = new Float32Array(buf, 12, n * 9);
    var geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(src.slice(), 3));
    geo.computeVertexNormals();
    geo.computeBoundingSphere();
    return geo;
  }

  function loadMeshes() {
    if (!meshPromise) meshPromise = Promise.all(MESH_NAMES.map(function (name) {
      return fetch("/static/meshes/" + name + ".bin", { cache: "force-cache" })
        .then(function (r) { return r.ok ? r.arrayBuffer() : null; })
        .then(function (buf) { return buf ? parseFarm(buf) : null; })
        .catch(function () { return null; });
    })).then(function (geos) {
      if (geos.filter(Boolean).length < 4) return null;
      var parts = splitLink6(geos[6]);
      if (parts) {
        if (parts.body !== geos[6]) geos[6].dispose();
        geos[6] = parts.body;
        geos.gripper = parts;
      }
      return geos;
    });
    return meshPromise;
  }

  function createArm3D(hostId, overlayId) {
  var GRIP_LO = 0;
  var GRIP_HI = 105;
  var JAW_TRAVEL = 0.042;
  var GRIP_T_PER_S = 2.5;
  var host = null, overlay = null, renderer = null, scene = null, camera = null;
  var robot = null, nodes = [], jointAxes = [];
  var jawL = null, jawR = null, jawLCad = null, jawRCad = null;
  var lastGrip = NaN, lastGripOpen = null, shownT = 1;
  var gripInited = false, gripClock = 0;
  var baseAxes = null, eeAxes = null;
  var raf = 0, timer = 0, lastRender = -Infinity;
  var active = false, ready = false, dirty = true, poseDirty = true, pathDirty = false, inTick = false;
  var az = 0.95, el = 0.38, dist = 1.15;
  var lookX = 0.04, lookY = 0.16, lookZ = 0;
  var dragging = false, lastX = 0, lastY = 0;
  var lastQ = [0, 0, 0, 0, 0, 0];
  var needsFit = true, cadOk = false, pathLine = null, pathFrames = [];
  var grid = null;

  function $(id) { return document.getElementById(id); }
  function setOverlay(text) {
    overlay = overlay || $(overlayId);
    if (overlay) overlay.textContent = text;
  }

  function bone(parent, to, radius, color) {
    var dir = new THREE.Vector3(to[0], to[1], to[2]);
    var len = dir.length();
    if (len < 1e-5) return;
    var geo = new THREE.CylinderGeometry(radius * 0.88, radius, len, 14);
    var mat = new THREE.MeshLambertMaterial({ color: color });
    var mesh = new THREE.Mesh(geo, mat);
    mesh.position.copy(dir).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
    parent.add(mesh);
  }

  function hub(parent, radius, color) {
    var mesh = new THREE.Mesh(
      new THREE.SphereGeometry(radius * 1.15, 16, 12),
      new THREE.MeshLambertMaterial({ color: color })
    );
    parent.add(mesh);
  }

  function triad(size) {
    var axes = new THREE.AxesHelper(size);
    colorTriad(axes);
    var mats = axes.material;
    if (Array.isArray(mats)) {
      mats.forEach(function (m) {
        if (m) m.depthTest = false;
      });
    } else if (mats) {
      mats.depthTest = false;
    }
    axes.renderOrder = 2;
    return axes;
  }
  function colorTriad(axes) {
    if (axes.setColors) axes.setColors(
      new THREE.Color(themeColor("--axis-x", 0xff8a96)),
      new THREE.Color(themeColor("--axis-y", 0x5dd59c)),
      new THREE.Color(themeColor("--axis-z", 0x70bcff)));
  }

  function lookAtTarget() {
    return new THREE.Vector3(lookX, lookY, lookZ);
  }

  function placeCamera() {
    if (!camera) return;
    var e = Math.max(-1.15, Math.min(1.25, el));
    var target = lookAtTarget();
    camera.position.set(
      target.x + dist * Math.cos(e) * Math.sin(az),
      target.y + dist * Math.sin(e),
      target.z + dist * Math.cos(e) * Math.cos(az)
    );
    camera.lookAt(target);
    invalidate();
  }

  function updateCameraFit() {
    if (!robot || !camera || !host) return;
    robot.updateMatrixWorld(true);
    var box = new THREE.Box3().setFromObject(robot);
    if (pathLine) box.union(new THREE.Box3().setFromObject(pathLine));
    if (!box.isEmpty()) {
      var sphere = box.getBoundingSphere(new THREE.Sphere());
      lookX = sphere.center.x;
      lookY = sphere.center.y;
      lookZ = sphere.center.z;
      var vFov = (camera.fov * Math.PI) / 180;
      var aspect = Math.max(0.35, camera.aspect || (host.clientWidth / Math.max(1, host.clientHeight)));
      var hFov = 2 * Math.atan(Math.tan(vFov / 2) * aspect);
      var fit = sphere.radius / Math.sin(Math.min(vFov, hFov) / 2);
      dist = Math.max(0.55, Math.min(2.8, fit * 1.38));
    }
    placeCamera();
    needsFit = false;
  }

  function fitCamera() {
    needsFit = true;
    invalidate();
  }

  function gripT() {
    var lo = GRIP_LO;
    var hi = GRIP_HI;
    var d = Number(lastGrip);
    if (isFinite(d) && hi > lo) {
      return Math.max(0, Math.min(1, (d - lo) / (hi - lo)));
    }
    if (lastGripOpen === false) return 0;
    if (lastGripOpen === true) return 1;
    return 1;
  }

  function applyGripVisual(t) {
    t = Math.max(0, Math.min(1, t));
    var close = 1 - t;
    if (jawLCad) jawLCad.position.y = -close * JAW_TRAVEL;
    if (jawRCad) jawRCad.position.y = close * JAW_TRAVEL;
    var spread = 0.005 + t * 0.02;
    if (jawL) jawL.position.y = spread;
    if (jawR) jawR.position.y = -spread;
  }

  function stepGrip(dt) {
    var tgt = gripT();
    if (!gripInited) {
      shownT = tgt;
      gripInited = true;
      applyGripVisual(shownT);
      return;
    }
    var maxd = GRIP_T_PER_S * Math.max(0, dt);
    var err = tgt - shownT;
    if (Math.abs(err) <= maxd) shownT = tgt;
    else shownT += err > 0 ? maxd : -maxd;
    applyGripVisual(shownT);
  }

  function applyJoints(qDeg) {
    var i;
    for (i = 0; i < nodes.length; i += 1) {
      var q = ((qDeg[i] || 0) * Math.PI) / 180;
      nodes[i].quaternion.setFromAxisAngle(jointAxes[i], q);
    }
  }

  function buildRobot(geos) {
    if (robot && scene) scene.remove(robot);
    robot = new THREE.Group();
    robot.rotation.x = -Math.PI / 2;
    var useCad = !!(geos && geos[0]);
    cadOk = useCad;
    jawLCad = null;
    jawRCad = null;
    jawL = null;
    jawR = null;

    if (useCad) {
      robot.add(new THREE.Mesh(geos[0], cadMaterial(MESH_COLORS[0])));
    } else {
      var base = new THREE.Mesh(
        new THREE.CylinderGeometry(0.055, 0.062, 0.036, 24),
        new THREE.MeshLambertMaterial({ color: 0x3a4654 })
      );
      base.rotation.x = Math.PI / 2;
      base.position.z = 0.018;
      robot.add(base);
    }
    baseAxes = triad(0.08);
    robot.add(baseAxes);

    var parent = robot;
    nodes = [];
    var i;
    for (i = 0; i < JOINTS.length; i += 1) {
      var spec = JOINTS[i];
      if (!useCad) bone(parent, spec.xyz, spec.r, spec.color);
      var node = new THREE.Group();
      node.position.set(spec.xyz[0], spec.xyz[1], spec.xyz[2]);
      parent.add(node);
      if (useCad && i === 5 && geos[6]) {
        var parts = geos.gripper;
        if (parts && parts.left && parts.right) {
          node.add(new THREE.Mesh(parts.body, cadMaterial(MESH_COLORS[6])));
          jawLCad = new THREE.Group();
          jawRCad = new THREE.Group();
          jawLCad.add(new THREE.Mesh(parts.left, cadMaterial(MESH_COLORS[6])));
          jawRCad.add(new THREE.Mesh(parts.right, cadMaterial(MESH_COLORS[6])));
          if (parts.travel) JAW_TRAVEL = parts.travel;
          node.add(jawLCad);
          node.add(jawRCad);
        } else {
          node.add(new THREE.Mesh(geos[6], cadMaterial(MESH_COLORS[6])));
        }
      } else if (useCad && geos[i + 1] && i !== 5) {
        node.add(new THREE.Mesh(geos[i + 1], cadMaterial(MESH_COLORS[i + 1])));
      } else if (!useCad) {
        hub(node, spec.r, i % 2 ? 0xd7f3ff : 0xc5d0db);
      }
      nodes.push(node);
      parent = node;
    }
    if (!useCad) bone(parent, TOOL, 0.011, 0x5ec8ff);

    var tool = new THREE.Group();
    tool.position.set(TOOL[0], TOOL[1], TOOL[2]);
    parent.add(tool);
    eeAxes = triad(0.07);
    tool.add(eeAxes);

    jawL = null;
    jawR = null;
    if (!jawLCad || !jawRCad) {
      var jawMat = new THREE.MeshLambertMaterial({ color: MESH_COLORS[6] });
      var padMat = new THREE.MeshLambertMaterial({ color: 0xa8a8a8 });
      var jawGeo = new THREE.BoxGeometry(0.055, 0.01, 0.018);
      var padGeo = new THREE.BoxGeometry(0.042, 0.004, 0.016);
      jawL = new THREE.Group();
      jawR = new THREE.Group();
      var mL = new THREE.Mesh(jawGeo, jawMat);
      var mR = new THREE.Mesh(jawGeo, jawMat);
      var pL = new THREE.Mesh(padGeo, padMat);
      var pR = new THREE.Mesh(padGeo, padMat);
      pL.position.set(0.004, -0.007, 0);
      pR.position.set(0.004, 0.007, 0);
      jawL.add(mL);
      jawL.add(pL);
      jawR.add(mR);
      jawR.add(pR);
      jawL.position.set(0.04, 0.02, 0);
      jawR.position.set(0.04, -0.02, 0);
      tool.add(jawL);
      tool.add(jawR);
    }

    scene.add(robot);
  }

  function hintText() {
    return cadOk ? "CAD 网格 · fafu_baseV1" : "简化模型 · 未加载 CAD 网格";
  }

  function onPointerDown(ev) {
    dragging = true;
    lastX = ev.clientX;
    lastY = ev.clientY;
    if (host) host.setPointerCapture(ev.pointerId);
  }
  function onPointerMove(ev) {
    if (!dragging) return;
    var dx = ev.clientX - lastX;
    var dy = ev.clientY - lastY;
    lastX = ev.clientX;
    lastY = ev.clientY;
    az -= dx * 0.008;
    el += dy * 0.008;
    placeCamera();
  }
  function onPointerUp(ev) {
    dragging = false;
    try {
      if (host) host.releasePointerCapture(ev.pointerId);
    } catch (e) {}
  }
  function onWheel(ev) {
    ev.preventDefault();
    dist = Math.max(0.4, Math.min(3.2, dist * (ev.deltaY > 0 ? 1.08 : 0.92)));
    placeCamera();
  }
  function onDblClick(ev) {
    ev.preventDefault();
    fitCamera();
  }

  function resize() {
    if (!active || activeView !== api || document.hidden || !host || !renderer || !camera) return;
    var w = host.clientWidth;
    var h = host.clientHeight;
    if (w < 8 || h < 8) return;
    var dpr = Math.min(window.devicePixelRatio || 1, 1.5);
    if (renderWidth !== w || renderHeight !== h || renderDpr !== dpr) {
      renderer.setPixelRatio(dpr);
      renderer.setSize(w, h, false);
      renderWidth = w; renderHeight = h; renderDpr = dpr;
      invalidate();
    }
    if (camera.aspect !== w / h) {
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      invalidate();
    }
  }

  function drawable() {
    return active && activeView === api && ready && !document.hidden &&
      host && host.clientWidth > 8 && host.clientHeight > 8;
  }

  function cancelPending() {
    if (raf) cancelAnimationFrame(raf);
    if (timer) clearTimeout(timer);
    raf = 0; timer = 0; gripClock = 0;
  }

  function schedule() {
    if (inTick || raf || timer || !drawable()) return;
    var wait = FRAME_MS - (performance.now() - lastRender);
    if (wait > 0) {
      timer = setTimeout(function () { timer = 0; schedule(); }, wait);
    } else raf = requestAnimationFrame(tick);
  }

  function invalidate() {
    dirty = true;
    schedule();
  }

  function applyTheme() {
    if (!scene) return;
    scene.traverse(function (object) { if (object.type === "AxesHelper") colorTriad(object); });
    scene.background.set(themeColor("--view-bg", 0x070b10));
    if (grid) {
      scene.remove(grid);
      grid.geometry.dispose();
      (Array.isArray(grid.material) ? grid.material : [grid.material]).forEach(function (material) { material.dispose(); });
    }
    grid = new THREE.GridHelper(1.4, 14,
      themeColor("--view-grid-major", 0x334556), themeColor("--view-grid-minor", 0x1b2733));
    scene.add(grid);
    if (pathLine) pathLine.material.color.set(themeColor("--cyan", 0x5ec8ff));
    invalidate();
  }

  function tick(now) {
    raf = 0;
    if (!drawable()) return;
    inTick = true;
    var dt = gripClock ? Math.min(0.05, (now - gripClock) / 1000) : 0.016;
    gripClock = now;
    if (pathDirty) updatePath();
    if (poseDirty) { applyJoints(lastQ); poseDirty = false; }
    stepGrip(dt);
    if (needsFit) updateCameraFit();
    if (dirty) {
      renderer.render(scene, camera);
      lastRender = now;
    }
    dirty = false;
    inTick = false;
    if (Math.abs(gripT() - shownT) > 0.00001) invalidate();
  }

  function setActive(on) {
    if (!on) {
      active = false;
      if (activeView === api) activeView = null;
      cancelPending();
      return;
    }
    if (activeView && activeView !== api) activeView.setActive(false);
    activeView = api;
    active = true;
    if (!init()) return;
    if (renderer.domElement.parentNode !== host) host.appendChild(renderer.domElement);
    resize();
    invalidate();
  }

  function init() {
    if (scene) return true;
    host = $(hostId);
    overlay = $(overlayId);
    if (!host || !active) return false;
    host.title = "拖拽旋转 · 滚轮缩放 · 双击框选";
    if (typeof THREE === "undefined") {
      setOverlay("未加载 Three.js");
      return false;
    }
    try {
      renderer = getRenderer();
    } catch (err) {
      setOverlay("本机无法创建 WebGL");
      return false;
    }
    scene = new THREE.Scene();
    scene.background = new THREE.Color(themeColor("--view-bg", 0x070b10));
    camera = new THREE.PerspectiveCamera(46, 1, 0.02, 20);
    jointAxes = JOINTS.map(function (spec) {
      return new THREE.Vector3(spec.axis[0], spec.axis[1], spec.axis[2]).normalize();
    });

    scene.add(new THREE.AmbientLight(0xffffff, 0.42));
    var key = new THREE.DirectionalLight(0xffffff, 0.95);
    key.position.set(0.6, 1.2, 0.8);
    scene.add(key);
    var fill = new THREE.DirectionalLight(0x8ab4c8, 0.4);
    fill.position.set(-0.8, 0.4, -0.5);
    scene.add(fill);
    var rim = new THREE.DirectionalLight(0xffffff, 0.22);
    rim.position.set(0.1, 0.2, -1.0);
    scene.add(rim);

    applyTheme();

    setOverlay("加载 CAD 网格…");
    loadMeshes().then(function (geos) {
      buildRobot(geos);
      shownT = gripT();
      gripInited = true;
      applyGripVisual(shownT);
      ready = true;
      setOverlay(hintText());
      // A view may have been hidden while loading. Do not steal the canvas
      // back or compute its latest pose/path until it is visible again.
      resize();
      invalidate();
    });

    host.addEventListener("pointerdown", onPointerDown);
    host.addEventListener("pointermove", onPointerMove);
    host.addEventListener("pointerup", onPointerUp);
    host.addEventListener("pointerleave", onPointerUp);
    host.addEventListener("wheel", onWheel, { passive: false });
    host.addEventListener("dblclick", onDblClick);
    window.addEventListener("resize", resize);
    if (typeof ResizeObserver !== "undefined") {
      new ResizeObserver(resize).observe(host);
    }
    document.addEventListener("visibilitychange", function () {
      if (document.hidden) cancelPending();
      else if (active) { resize(); invalidate(); }
    });
    return true;
  }

  function setGripRange(lo, hi) {
    var a = Number(lo);
    var b = Number(hi);
    if (!isFinite(a) || !isFinite(b) || b <= a) return;
    if (a === GRIP_LO && b === GRIP_HI) return;
    GRIP_LO = a;
    GRIP_HI = b;
    invalidate();
  }

  function setPose(qDeg, gripDeg, gripOpen) {
    var changed = false;
    if (qDeg) for (var i = 0; i < JOINTS.length; i += 1) {
      var q = Number(qDeg[i]);
      if (!isFinite(q)) q = 0;
      if (lastQ[i] !== q) { lastQ[i] = q; poseDirty = true; changed = true; }
    }
    if (gripDeg != null && gripDeg !== "" && isFinite(Number(gripDeg)) && lastGrip !== Number(gripDeg)) {
      lastGrip = Number(gripDeg); changed = true;
    }
    if ((gripOpen === true || gripOpen === false) && lastGripOpen !== gripOpen) {
      lastGripOpen = gripOpen; changed = true;
    }
    if (changed) invalidate();
  }

  function setPath(frames) {
    if (frames === pathFrames) return;
    pathFrames = frames || [];
    pathDirty = true;
    invalidate();
  }

  function updatePath() {
    pathDirty = false;
    if (pathLine) {
      scene.remove(pathLine);
      pathLine.geometry.dispose();
      pathLine.material.dispose();
      pathLine = null;
    }
    needsFit = true;
    if (pathFrames.length < 2) return;
    var points = [];
    pathFrames.forEach(function (frame) {
      applyJoints(frame.pos.map(function (q) { return q * 180 / Math.PI; }));
      robot.updateMatrixWorld(true);
      points.push(eeAxes.getWorldPosition(new THREE.Vector3()));
    });
    pathLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({color: themeColor("--cyan", 0x5ec8ff), transparent: true, opacity: 0.7}));
    scene.add(pathLine);
    applyJoints(lastQ);
  }

  window.addEventListener("station:theme", applyTheme);
  var api = {
    init: init,
    setPose: setPose,
    setGripRange: setGripRange,
    resize: resize,
    setActive: setActive,
    fitCamera: fitCamera,
    setPath: setPath,
  };
  return api;
  }

  global.Arm3D = createArm3D("arm3dHost", "arm3dOverlay");
  global.Arm3D.create = createArm3D;
  // Navigation activates a view on demand; hidden workspaces allocate no GL resources.
})(window);
