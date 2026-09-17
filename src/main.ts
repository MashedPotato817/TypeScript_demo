import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { forwardKinematics, formatJoint, initialQ, joints, solvePositionIK } from './robot';
import { buildShapeTrajectory, shapeLabels, type ShapeName, type Trajectory } from './trajectory';
import './style.css';

const unitScale = 0.55;
const app = document.querySelector<HTMLDivElement>('#app')!;

app.innerHTML = `
  <header class="topbar"><div><span class="eyebrow">ROBOTICS / LAB 08</span><h1>八自由度机械臂 <em>运动学实验台</em></h1></div><div class="mode-readout"><span class="pulse"></span><span id="modeLabel">关节控制</span><small>DH · FK · IK · TRAJECTORY</small></div></header>
  <main>
    <section class="stage"><div id="viewport" aria-label="机械臂三维视图"></div><div class="canvas-note"><b>工作空间</b><span>点击地面可设定末端目标点</span></div><div class="legend"><i class="x"></i>X <i class="y"></i>Y <i class="z"></i>Z <i class="path"></i>轨迹</div></section>
    <aside class="dashboard">
      <div class="tabs"><button class="tab active" data-panel="joint">关节控制</button><button class="tab" data-panel="target">末端目标</button><button class="tab" data-panel="path">轨迹任务</button></div>
      <div id="panel-joint" class="panel active"><p class="hint">滑动关节变量，实时计算末端位姿。</p><div id="jointControls"></div><button class="button quiet" id="resetButton">恢复实验初始位姿</button></div>
      <div id="panel-target" class="panel"><p class="hint">输入目标位置或点击工作台。逆解只约束位置，以当前姿态为初值。</p><div class="target-grid"><label>X<input id="targetX" type="number" step="0.1" value="3.0" /></label><label>Y<input id="targetY" type="number" step="0.1" value="3.0" /></label><label>Z<input id="targetZ" type="number" step="0.1" value="8.0" /></label></div><button class="button" id="solveButton">求解当前位置</button><p id="ikStatus" class="status">等待目标点</p></div>
      <div id="panel-path" class="panel"><p class="hint">末端沿笛卡尔轨迹作图。每一帧均锁定末端姿态，青色轨迹由实际播放关节角的正运动学计算。</p><label class="shape-picker">轨迹形状<select id="shapeSelect"><option value="circle">圆形</option><option value="square">正方形</option><option value="rectangle">矩形</option><option value="heart">心形</option></select></label><div class="path-meta"><span id="shapeName">圆形</span><span id="pathProgress">0 / 144 帧</span><span id="pathAccuracy">计算中</span></div><div class="button-row"><button class="button" id="playButton">播放路径</button><button class="button quiet" id="resetPathButton">重置</button></div></div>
      <section class="telemetry"><div class="section-title"><span>末端遥测</span><small>BASE → TOOL</small></div><div class="coordinates"><div><small>X</small><strong id="coordX">—</strong></div><div><small>Y</small><strong id="coordY">—</strong></div><div><small>Z</small><strong id="coordZ">—</strong></div></div><div class="matrix-header">齐次变换矩阵 <span>T<sub>0,8</sub></span></div><pre id="matrix"></pre></section>
    </aside>
  </main>
`;

const viewport = document.querySelector<HTMLDivElement>('#viewport')!;
const scene = new THREE.Scene();
scene.background = new THREE.Color('#101614');
scene.fog = new THREE.Fog('#101614', 18, 46);
const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
camera.position.set(15, 13, 19);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
viewport.append(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 4, 0);
controls.enableDamping = true;
controls.minDistance = 11;
controls.maxDistance = 42;

scene.add(new THREE.HemisphereLight('#d4f4d6', '#050807', 2));
const key = new THREE.DirectionalLight('#eaf8d7', 2.3); key.position.set(7, 14, 8); key.castShadow = true; scene.add(key);
const grid = new THREE.GridHelper(36, 36, '#34493e', '#1c2923'); grid.position.y = -0.03; scene.add(grid);
scene.add(new THREE.AxesHelper(2.2));

const robotGroup = new THREE.Group(); scene.add(robotGroup);
const linkMaterial = new THREE.MeshStandardMaterial({ color: '#c4d09b', metalness: 0.65, roughness: 0.32 });
const jointMaterial = new THREE.MeshStandardMaterial({ color: '#ef8f38', metalness: 0.4, roughness: 0.35 });
const links = Array.from({ length: 8 }, () => { const mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.21, 1, 12), linkMaterial); mesh.castShadow = true; robotGroup.add(mesh); return mesh; });
const nodes = Array.from({ length: 9 }, () => { const mesh = new THREE.Mesh(new THREE.SphereGeometry(0.28, 18, 14), jointMaterial); mesh.castShadow = true; robotGroup.add(mesh); return mesh; });
const endAxes = new THREE.AxesHelper(1.4); robotGroup.add(endAxes);
const targetMarker = new THREE.Mesh(new THREE.SphereGeometry(0.25, 18, 14), new THREE.MeshBasicMaterial({ color: '#68e6ce' })); targetMarker.visible = false; scene.add(targetMarker);
const trajectoryLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#72d7d0' })); scene.add(trajectoryLine);

const mapPoint = (point: THREE.Vector3) => new THREE.Vector3(point.x * unitScale, point.z * unitScale, -point.y * unitScale);
const mapQuaternion = (matrix: THREE.Matrix4) => new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeRotationX(Math.PI / 2).multiply(matrix));
function updateCylinder(mesh: THREE.Mesh, from: THREE.Vector3, to: THREE.Vector3): void {
  const direction = to.clone().sub(from); const length = direction.length();
  mesh.visible = length > 0.02;
  if (!mesh.visible) return;
  mesh.position.copy(from.clone().add(to).multiplyScalar(0.5));
  mesh.scale.set(1, length, 1);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
}

let q = [...initialQ];
let target = new THREE.Vector3(3, 3, 8);
let trajectory: Trajectory = buildShapeTrajectory('circle');
let trajectoryIndex = 0;
let playing = false;
let frameAccumulator = 0;

const controlRoot = document.querySelector<HTMLDivElement>('#jointControls')!;
const sliders: HTMLInputElement[] = [];
joints.forEach((joint, index) => {
  const row = document.createElement('label'); row.className = 'joint-row';
  const min = joint.kind === 'prismatic' ? joint.min : THREE.MathUtils.radToDeg(joint.min);
  const max = joint.kind === 'prismatic' ? joint.max : THREE.MathUtils.radToDeg(joint.max);
  row.innerHTML = `<span><b>${joint.name}</b><small>${joint.kind === 'prismatic' ? 'P · u' : 'R · °'}</small></span><input type="range" min="${min}" max="${max}" step="${joint.kind === 'prismatic' ? '0.01' : '1'}" /><output></output>`;
  const slider = row.querySelector<HTMLInputElement>('input')!;
  const output = row.querySelector<HTMLOutputElement>('output')!;
  slider.addEventListener('input', () => { q[index] = joint.kind === 'prismatic' ? Number(slider.value) : THREE.MathUtils.degToRad(Number(slider.value)); playing = false; refresh(); });
  sliders.push(slider); controlRoot.append(row);
  output.textContent = formatJoint(q[index], index);
});

function setTargetInputs(): void { (document.querySelector<HTMLInputElement>('#targetX')!).value = target.x.toFixed(2); (document.querySelector<HTMLInputElement>('#targetY')!).value = target.y.toFixed(2); (document.querySelector<HTMLInputElement>('#targetZ')!).value = target.z.toFixed(2); }
function refresh(): void {
  const result = forwardKinematics(q);
  const worldPoints = result.points.map(mapPoint);
  nodes.forEach((node, index) => node.position.copy(worldPoints[index]));
  links.forEach((link, index) => updateCylinder(link, worldPoints[index], worldPoints[index + 1]));
  endAxes.position.copy(worldPoints.at(-1)!); endAxes.quaternion.copy(mapQuaternion(result.transform));
  sliders.forEach((slider, index) => { slider.value = String(joints[index].kind === 'prismatic' ? q[index] : THREE.MathUtils.radToDeg(q[index])); slider.parentElement!.querySelector('output')!.textContent = formatJoint(q[index], index); });
  const endpoint = result.points.at(-1)!;
  (document.querySelector('#coordX')!).textContent = endpoint.x.toFixed(2); (document.querySelector('#coordY')!).textContent = endpoint.y.toFixed(2); (document.querySelector('#coordZ')!).textContent = endpoint.z.toFixed(2);
  const elements = result.transform.elements;
  (document.querySelector('#matrix')!).textContent = [0, 1, 2, 3].map(row => [0, 1, 2, 3].map(col => elements[col * 4 + row].toFixed(3).padStart(8)).join(' ')).join('\n');
  document.querySelector('#pathProgress')!.textContent = `${trajectoryIndex} / ${trajectory.samples.length} 帧`;
}

function renderTrajectoryLine(): void {
  // Exactly the same joint samples drive both the path line and the animation.
  trajectoryLine.geometry.setFromPoints(trajectory.actualPoints.map(mapPoint));
  document.querySelector('#pathAccuracy')!.textContent = `最大误差 ${trajectory.maxPositionError.toFixed(3)} u`;
}

function selectShape(shape: ShapeName): void {
  playing = false;
  trajectory = buildShapeTrajectory(shape);
  trajectoryIndex = 0;
  q = [...trajectory.samples[0]];
  document.querySelector('#shapeName')!.textContent = shapeLabels[shape];
  document.querySelector('#playButton')!.textContent = '播放路径';
  renderTrajectoryLine();
  refresh();
}

renderTrajectoryLine();

document.querySelector('#resetButton')!.addEventListener('click', () => { q = [...initialQ]; playing = false; refresh(); });
document.querySelector('#solveButton')!.addEventListener('click', () => {
  target.set(Number((document.querySelector('#targetX') as HTMLInputElement).value), Number((document.querySelector('#targetY') as HTMLInputElement).value), Number((document.querySelector('#targetZ') as HTMLInputElement).value));
  targetMarker.position.copy(mapPoint(target)); targetMarker.visible = true;
  const solved = solvePositionIK(target, q); q = solved.q; playing = false;
  const status = document.querySelector('#ikStatus')!; status.textContent = `${solved.converged ? '已收敛' : '未完全收敛'} · 误差 ${solved.error.toFixed(3)} u · ${solved.iterations} 次`;
  status.className = `status ${solved.converged ? 'success' : 'warning'}`; refresh();
});
document.querySelector('#playButton')!.addEventListener('click', () => { playing = !playing; document.querySelector('#playButton')!.textContent = playing ? '暂停路径' : '继续播放'; });
document.querySelector('#resetPathButton')!.addEventListener('click', () => { trajectoryIndex = 0; q = [...trajectory.samples[0]]; playing = false; document.querySelector('#playButton')!.textContent = '播放路径'; refresh(); });
document.querySelector<HTMLSelectElement>('#shapeSelect')!.addEventListener('change', event => selectShape((event.currentTarget as HTMLSelectElement).value as ShapeName));
document.querySelectorAll<HTMLButtonElement>('.tab').forEach(tab => tab.addEventListener('click', () => { document.querySelectorAll('.tab').forEach(item => item.classList.remove('active')); document.querySelectorAll('.panel').forEach(item => item.classList.remove('active')); tab.classList.add('active'); document.querySelector(`#panel-${tab.dataset.panel}`)!.classList.add('active'); document.querySelector('#modeLabel')!.textContent = tab.textContent ?? ''; }));

const raycaster = new THREE.Raycaster(); const pointer = new THREE.Vector2(); const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
renderer.domElement.addEventListener('pointerdown', event => {
  if (event.button !== 0 || event.shiftKey) return;
  const rect = renderer.domElement.getBoundingClientRect(); pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(pointer, camera); const hit = new THREE.Vector3();
  if (raycaster.ray.intersectPlane(ground, hit)) { target.set(hit.x / unitScale, -hit.z / unitScale, hit.y / unitScale); setTargetInputs(); targetMarker.position.copy(hit); targetMarker.visible = true; document.querySelector<HTMLButtonElement>('[data-panel="target"]')!.click(); }
});

function resize(): void { const { width, height } = viewport.getBoundingClientRect(); camera.aspect = width / height; camera.updateProjectionMatrix(); renderer.setSize(width, height, false); }
new ResizeObserver(resize).observe(viewport); resize(); refresh();
let previous = performance.now();
function animate(now: number): void { const elapsed = now - previous; previous = now; if (playing) { frameAccumulator += elapsed; if (frameAccumulator > 1000 / 72) { frameAccumulator = 0; q = [...trajectory.samples[trajectoryIndex]]; trajectoryIndex += 1; if (trajectoryIndex >= trajectory.samples.length) { trajectoryIndex = 0; playing = false; document.querySelector('#playButton')!.textContent = '播放路径'; } refresh(); } } controls.update(); renderer.render(scene, camera); requestAnimationFrame(animate); }
requestAnimationFrame(animate);
