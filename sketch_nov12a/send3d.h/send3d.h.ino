 void send3DPage(String connId) {
  String html = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
  html += R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Rocket Orientation</title>
  <style>
  body {
    margin: 0;
    background: #111;
    display: flex;
    align-items: center;
    justify-content: center;
    height: 100vh;
  }
  #viewer-box {
    width: 600px;
    height: 400px;
    border: 2px solid #0f0;
    background: #000;
    box-shadow: 0 0 20px #0f0;
  }
  canvas {
    width: 100%;
    height: 100%;
    display: block;
  }
</style>

</head>
<body>
<script src="https://cdn.jsdelivr.net/npm/three@0.157.0/build/three.min.js"></script>
<script>
let scene = new THREE.Scene();
let camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
let renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
let container = document.createElement('div');
container.id = 'viewer-box';
document.body.appendChild(container);
container.appendChild(renderer.domElement);

 

// Rocket body
let rocket = new THREE.Group();

// Cylinder body
let geometry = new THREE.CylinderGeometry(0.5, 0.5, 4, 32);
let material = new THREE.MeshStandardMaterial({color: 0x00ff00});
let cylinder = new THREE.Mesh(geometry, material);
rocket.add(cylinder);

// Cone tip
let coneGeo = new THREE.ConeGeometry(0.5, 1, 32);
let coneMat = new THREE.MeshStandardMaterial({color: 0xff0000});
let cone = new THREE.Mesh(coneGeo, coneMat);
cone.position.y = 2.5;
rocket.add(cone);

scene.add(rocket);

let light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(10, 10, 10).normalize();
scene.add(light);

camera.position.z = 10;

// Rotation variables
let roll = 0, pitch = 0, yaw = 0;

function animate() {
  requestAnimationFrame(animate);
  rocket.rotation.x = pitch;
  rocket.rotation.y = yaw;
  rocket.rotation.z = roll;
  renderer.render(scene, camera);
}
animate();

// Polling from Teensy every 300ms
setInterval(() => {
  fetch('/data')
    .then(res => res.json())
    .then(data => {
      roll = data.roll * Math.PI / 180;
      pitch = data.pitch * Math.PI / 180;
      yaw = data.yaw * Math.PI / 180;
    });
}, 300);
</script>
</body>
</html>
)rawliteral";

  String cmd = "AT+CIPSEND=" + connId + "," + String(html.length());
  sendAT(cmd);
  delay(100);
  espSerial.print(html);
  delay(100);
  sendAT("AT+CIPCLOSE=" + connId);
}