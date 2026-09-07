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
  var GRIP_LO = 0;
  var GRIP_HI = 105;
  var JAW_TRAVEL = 0.042;
  var GRIP_T_PER_S = 2.5;

  var host = null;
  var overlay = null;
  var renderer = null;
  var scene = null;
  var camera = null;
  var robot = null;
  var nodes = [];
  var jawL = null;
  var jawR = null;
  var jawLCad = null;
  var jawRCad = null;
  var lastGrip = NaN;
  var lastGripOpen = null;
  var shownT = 1;
  var gripInited = false;
  var gripClock = 0;
  var baseAxes = null;
  var eeAxes = null;
  var raf = 0;
  var active = false;
  var ready = false;
  var az = 0.95;
  var el = 0.38;
  var dist = 1.15;
  var lookX = 0.04;
  var lookY = 0.16;
  var lookZ = 0;
  var dragging = false;
  var lastX = 0;
  var lastY = 0;
  var lastQ = [0, 0, 0, 0, 0, 0];
  var needsFit = true;
  var cadOk = false;

  function $(id) {
    return document.getElementById(id);
  }

  function setOverlay(text) {
    overlay = overlay || $("arm3dOverlay");
    if (overlay) overlay.textContent = text;
  }

  function cadMaterial(color) {
    return new THREE.MeshPhongMaterial({
      color: color,
      specular: 0x2a3038,
      shininess: 48,
      side: THREE.DoubleSide,
      flatShading: true,
    });
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
    var travel = JAW_TRAVEL;
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

  function loadMeshes(done) {
    var left = MESH_NAMES.length;
    var geos = new Array(MESH_NAMES.length);
    var i;
    function finish() {
      left -= 1;
      if (left > 0) return;
      var ok = 0;
      for (i = 0; i < geos.length; i += 1) if (geos[i]) ok += 1;
      done(ok >= 4 ? geos : null);
    }
    for (i = 0; i < MESH_NAMES.length; i += 1) {
      (function (idx, name) {
        fetch("/static/meshes/" + name + ".bin", { cache: "force-cache" })
          .then(function (r) { return r.ok ? r.arrayBuffer() : null; })
          .then(function (buf) {
            geos[idx] = buf ? parseFarm(buf) : null;
            finish();
          })
          .catch(function () {
            geos[idx] = null;
            finish();
          });
      })(i, MESH_NAMES[i]);
    }
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
  }

  function fitCamera() {
    if (!robot || !camera || !host) return;
    robot.updateMatrixWorld(true);
    var box = new THREE.Box3().setFromObject(robot);
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
      var ax = JOINTS[i].axis;
      nodes[i].quaternion.setFromAxisAngle(
        new THREE.Vector3(ax[0], ax[1], ax[2]).normalize(),
        q
      );
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
        var parts = splitLink6(geos[6]);
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
    return (cadOk ? "CAD 网格 · fafu_baseV1" : "URDF 同链 · 未加载 STL，用几何体") +
      " · 拖拽旋转 · 滚轮缩放 · 双击框选";
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
    if (!host || !renderer || !camera) return;
    var w = host.clientWidth;
    var h = host.clientHeight;
    if (w < 8 || h < 8) return;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  function tick() {
    raf = 0;
    if (!ready) return;
    var now = typeof performance !== "undefined" ? performance.now() : Date.now();
    var dt = gripClock ? Math.min(0.05, (now - gripClock) / 1000) : 0.016;
    gripClock = now;
    stepGrip(dt);
    if (active && renderer && scene && camera && host && host.clientWidth > 8) {
      renderer.render(scene, camera);
    }
    if (active) raf = requestAnimationFrame(tick);
  }

  function setActive(on) {
    active = !!on;
    if (active && !raf) raf = requestAnimationFrame(tick);
  }

  function init() {
    host = $("arm3dHost");
    overlay = $("arm3dOverlay");
    if (!host) return false;
    if (typeof THREE === "undefined") {
      setOverlay("未加载 Three.js");
      return false;
    }
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070b10);
    camera = new THREE.PerspectiveCamera(46, 1, 0.02, 20);
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    } catch (err) {
      setOverlay("本机无法创建 WebGL");
      return false;
    }
    renderer.setClearColor(0x070b10, 1);
    renderer.domElement.style.display = "block";
    renderer.domElement.style.width = "100%";
    renderer.domElement.style.height = "100%";
    host.appendChild(renderer.domElement);

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

    var grid = new THREE.GridHelper(1.4, 14, 0x334556, 0x1b2733);
    grid.position.y = 0;
    scene.add(grid);

    setOverlay("加载 CAD 网格…");
    loadMeshes(function (geos) {
      buildRobot(geos);
      applyJoints(lastQ);
      shownT = gripT();
      gripInited = true;
      applyGripVisual(shownT);
      resize();
      fitCamera();
      ready = true;
      setOverlay(hintText());
      setActive(true);
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
    return true;
  }

  function setGripRange(lo, hi) {
    var a = Number(lo);
    var b = Number(hi);
    if (!isFinite(a) || !isFinite(b) || b <= a) return;
    GRIP_LO = a;
    GRIP_HI = b;
    if (ready) applyJoints(lastQ);
  }

  function setPose(qDeg, gripDeg, gripOpen) {
    lastQ = qDeg || lastQ;
    if (gripDeg != null && gripDeg !== "" && !isNaN(Number(gripDeg))) lastGrip = Number(gripDeg);
    if (gripOpen === true || gripOpen === false) lastGripOpen = gripOpen;
    if (!ready) return;
    applyJoints(lastQ);
    if (needsFit) fitCamera();
  }

  global.Arm3D = {
    init: init,
    setPose: setPose,
    setGripRange: setGripRange,
    resize: resize,
    setActive: setActive,
    fitCamera: fitCamera,
  };

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})(window);
