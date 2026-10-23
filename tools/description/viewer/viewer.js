import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';

const element = id => document.getElementById(id);
const viewport = element('viewport');
const status = element('status');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#0b1220');
const camera = new THREE.PerspectiveCamera(40, 1, 0.001, 100);
camera.up.set(0, 0, 1);
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
} catch (error) {
  status.textContent = 'WebGL 初始化失败，请在支持 WebGL 的 Chromium/Chrome 中打开。';
  throw error;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
viewport.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
const render = () => renderer.render(scene, camera);
controls.addEventListener('change', render);
scene.add(new THREE.HemisphereLight(0xdceeff, 0x334255, 2.8));
const light = new THREE.DirectionalLight(0xffffff, 3.0);
light.position.set(2, -3, 4);
scene.add(light);
const fill = new THREE.DirectionalLight(0x90baff, 1.5);
fill.position.set(-2, 2, 1);
scene.add(fill);
const grid = new THREE.GridHelper(2, 20, 0x40516a, 0x243347);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.005;
scene.add(grid);

let robot = null;
let tcpAxes = null;
const fields = new Map();
function fit() {
  if (!robot) return;
  robot.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(robot);
  const center = bounds.getCenter(new THREE.Vector3());
  const span = Math.max(...bounds.getSize(new THREE.Vector3()).toArray(), 0.05);
  const distance = span / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) * 1.5;
  camera.position.copy(center).add(new THREE.Vector3(1, -1.4, 0.9).normalize().multiplyScalar(distance));
  controls.target.copy(center);
  camera.near = span / 1000;
  camera.far = Math.max(100, span * 100);
  camera.updateProjectionMatrix();
  grid.scale.setScalar(span > 0.5 ? 1 : 0.2);
  controls.update();
  render();
}

function displayMode() {
  if (!robot) return;
  const mode = element('mode').value;
  robot.traverse(object => {
    if (object.isURDFVisual) object.visible = mode !== 'collision';
    if (object.isURDFCollider) object.visible = mode !== 'visual';
  });
  if (tcpAxes) tcpAxes.visible = element('tcp').checked;
  render();
}

function setJoint(name, value) {
  if (!robot) return;
  robot.setJointValue(name, value);
  const field = fields.get(name);
  if (field) {
    field.input.value = robot.joints[name].jointValue[0];
    field.output.value = `${robot.joints[name].jointValue[0].toFixed(3)} rad`;
  }
  robot.updateMatrixWorld(true);
  render();
}

function jointControls() {
  element('joints').replaceChildren();
  fields.clear();
  const joints = Object.entries(robot.joints).filter(([, joint]) =>
    joint.jointType === 'revolute' && !joint.mimicJoint);
  joints.sort(([a], [b]) => (a === 'left_outer_knuckle_joint' ? -1 : b === 'left_outer_knuckle_joint' ? 1 : a.localeCompare(b)));
  for (const [name, joint] of joints) {
    const wrapper = document.createElement('div');
    wrapper.className = 'joint';
    const label = document.createElement('label');
    const input = document.createElement('input');
    input.type = 'range';
    input.id = `joint-${name}`;
    input.min = joint.limit.lower;
    input.max = joint.limit.upper;
    input.step = '0.001';
    input.value = joint.jointValue[0];
    const output = document.createElement('output');
    output.value = `${joint.jointValue[0].toFixed(3)} rad`;
    label.htmlFor = input.id;
    label.append(document.createTextNode(name === 'left_outer_knuckle_joint' ? '夹爪开合' : name), output);
    wrapper.append(label, input);
    element('joints').append(wrapper);
    fields.set(name, { input, output });
    input.addEventListener('input', () => setJoint(name, Number(input.value)));
  }
}

function release(model) {
  scene.remove(model);
  model.traverse(object => {
    object.geometry?.dispose();
    const materials = object.material ? [object.material].flat() : [];
    for (const material of materials) material.dispose();
  });
}

function load() {
  element('model').disabled = true;
  element('joints').replaceChildren();
  status.textContent = '正在加载 URDF 与网格…';
  if (robot) release(robot);
  robot = null;
  tcpAxes = null;
  const manager = new THREE.LoadingManager();
  const failures = [];
  manager.onError = url => failures.push(url);
  manager.onLoad = () => {
    element('model').disabled = false;
    if (!robot || failures.length) {
      status.textContent = `加载失败：${failures.join(', ') || 'URDF 无法解析'}`;
      return;
    }
    robot.traverse(object => {
      if (object.isURDFCollider) object.traverse(child => {
        if (child.isMesh) child.material = new THREE.MeshStandardMaterial({
          color: 0xffb663, transparent: true, opacity: 0.48, depthWrite: false,
        });
      });
    });
    tcpAxes = new THREE.AxesHelper(0.045);
    robot.links.grasp_link.add(tcpAxes);
    jointControls();
    displayMode();
    fit();
    status.textContent = `模型已加载 · ${Object.keys(robot.links).length} links · ${Object.keys(robot.joints).length} joints`;
  };
  const loader = new URDFLoader(manager);
  loader.parseCollision = true;
  loader.load(element('model').value, model => {
    robot = model;
    scene.add(robot);
  }, undefined, error => {
    element('model').disabled = false;
    status.textContent = `模型加载失败：${error.message}`;
  });
}

new ResizeObserver(() => {
  const { width, height } = viewport.getBoundingClientRect();
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
  render();
}).observe(viewport);
element('model').addEventListener('change', load);
element('mode').addEventListener('change', displayMode);
element('tcp').addEventListener('change', displayMode);
element('fit').addEventListener('click', fit);
element('reset').addEventListener('click', () => {
  for (const name of fields.keys()) setJoint(name, 0);
});
for (const button of document.querySelectorAll('[data-grip]')) {
  button.addEventListener('click', () => setJoint('left_outer_knuckle_joint', Number(button.dataset.grip)));
}
load();
