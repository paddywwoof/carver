//<script type="javascript">
var ctx = document.getElementById("myCanvas").getContext("2d");
var W = 0.98 * Math.max(document.documentElement.clientWidth, window.innerWidth || 0);
var H = 0.6 * Math.max(document.documentElement.clientHeight, window.innerHeight || 0);
ctx.canvas.width  = W;
ctx.canvas.height = H;
var grdnt;
generateRainbow();
ctx.strokeStyle = "red";
ctx.font = "30px Arial";
var diptxt = document.querySelector('.diptxt');
var logtxt = document.querySelector('.logtxt');
logtxt.innerHTML = "";

var xrs = [], yrs = [];
var zcrs = [], zsrs = []; // hold as cos and sin to avoid jump 359 -> 0
var xs = [], ys = [], zs = [];
var xadj = 0.0, yadj = 0.0, zadj = 0.0; // angle offsets
var gxadj = 0.0, gyadj = 0.0; // g component offsets

var g, g_prev; // unit vector straight down
var h; // unit vector horizontal in plane of phone
var m, m_prev; // unit vector pointing magnetic north horizontally
var stk = 0.0;
var stk180 = 0.0;
var dip = 0.0;
var cpoints = ["N","NE","E","SE","S","SW","W","NW"];
var latitude; var longitude; var altitude;

function generateRainbow() {
  ////////////////////////////////////////////////////////////////
  grdnt = ctx.createLinearGradient(0, -0.5 * H, 0, 0.5 * H);
  for (var i=0; i<6; i++) {
    var clr = '#';
    for (var j=0; j<3; j++) {clr += Math.floor((Math.random() * 239.999) + 16).toString(16);}
    grdnt.addColorStop(i / 6.0, clr); 
  }
  ctx.fillStyle = grdnt;
}

function logReading() {
  ////////////////////////////////////////////////////////////////
  var dt = new Date();
  //navigator.geolocation.getCurrentPosition(onSuccess, onError); //doesn't seem to work on android
  //watchPosition set instead
  logtxt.innerHTML += " strike: " + stk.toFixed(1)
                   + stk180 + " dip: " + dip.toFixed(1) + "\n.... "
                   + dt.toLocaleString()
                   + " lat: " + latitude + " lon: " + longitude + " alt: " + altitude + "\n";
  generateRainbow();
}

function calibrate() {
  ////////////////////////////////////////////////////////////////
  xadj = medianMean(xrs, 0.0);
  yadj = medianMean(yrs, 0.0);
  zadj = Math.atan2(medianMean(zsrs, -1.0), medianMean(zcrs, 0.0)) + 0.5 * Math.PI;
  gxadj += g[0];
  gyadj += g[1];
}

function medianMean(arr, val) {
  // return median of values /////////////////////////////////////
  var NUM = 25;
  var END = 3;
  if (arr.unshift(val) > NUM) {
    arr.pop();
  }
  else if (arr.length < (2 * END + 1)) {
    return val;
  }
  var newarr = arr.slice();  // slice() to create clone otherwise sorts in place
  sum = 0.0;
  for (i=END; i<(newarr.length - END); i++) {
    sum += newarr[i];
  }
  return (sum / (newarr.length - 2 * END));
}

function dot(a, b) {
  // dot product two 3D vectors //////////////////////////////////
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  // cross product two 3D vectors ////////////////////////////////
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function triple_product(a, b, c) {
  // triple product (a X b) . c //////////////////////////////////
  return a[1] * b[2] * c[0] - a[2] * b[1] * c[0] + 
         a[2] * b[0] * c[1] - a[0] * b[2] * c[1] +
         a[0] * b[1] * c[2] - a[1] * b[0] * c[2];
}

function norm(a) {
  // normalize a 3D vector ///////////////////////////////////////
  var mag = Math.sqrt(Math.pow(a[0], 2) + Math.pow(a[1], 2) + Math.pow(a[2], 2));
  return [a[0] / mag, a[1] / mag, a[2] / mag];
}

function taitBryan(a, x, y, z) {
  // rotate 3D vector a using Tait Bryan Z,Y',X'' x,y,z in degrees
  ////////////////////////////////////////////////////////////////
  x *= -Math.PI / 180.0;
  y *= -Math.PI / 180.0;
  z *= Math.PI / 180.0;
  sx = Math.sin(x);
  cx = Math.cos(x);
  sy = Math.sin(y);
  cy = Math.cos(y);
  sz = Math.sin(z);
  cz = Math.cos(z);
  return [(cy * cz) * a[0]                + (-cy * sz) * a[1]               + (sy) * a[2],
          (cx * sz + cz * sx * sy) * a[0] + (cx * cz - sx * sy * sz) * a[1] + (-cy * sx) * a[2],
          (sx * sz - cx * cz * sy) * a[0] + (cz * sx + cx * sy * sz) * a[1] + (cx * cy) * a[2]]
}

function handleOrientation(event) {
  // rotation accelerometer and magnetometer changes /////////////
  ////////////////////////////////////////////////////////////////
  var x = medianMean(xrs, event.beta) - xadj;
  var y = medianMean(yrs, event.gamma) - yadj;
  var a = event.alpha * Math.PI / 180.0;
  var z = Math.atan2(medianMean(zsrs, Math.sin(a)), medianMean(zcrs, Math.cos(a))) - zadj;
  z = (630.0 - z * 180.0 / Math.PI) % 360.0;
  m = taitBryan([0, -1, 0], x, y, z);
  g = taitBryan([0, 0, 1], x, y, z);
  if (m_prev !== undefined) {
    var rot = triple_product(m, m_prev, g);
    diptxt.innerHTML = "- rotation: " + rot.toFixed(1);
  }
  m_prev = m;
  /*
  h = norm([-g[1], g[0], 0]);
  dip = Math.sqrt((Math.pow(g[0], 2) + Math.pow(g[1], 2)) /
               (Math.pow(g[0], 2) + Math.pow(g[1], 2) + Math.pow(g[2], 2)));
  dip = 90.0 - Math.acos(dip) * 180.0 / Math.PI;
  stk = (450.0 - Math.atan2(dot(m, h), dot(m, cross(g, h))) * 180.0 / Math.PI) % 360.0;
  stk180 = " (" + (stk > 180.0 ? stk - 180.0 : stk).toFixed(1) + 
               " " + cpoints[Math.floor((stk + 112.5) / 45.0) % 8] + ")";
  diptxt.innerHTML = "- strike: " + stk.toFixed(1) + stk180 +
                     "\n- dip: " + dip.toFixed(1);

  x = Math.max(-90, Math.min(90, x));
  x += 90;
  y += 90;

  var ngl = Math.atan2(-h[1], -h[0]);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, W, H);

  ctx.translate(W/2, H/2);
  ctx.rotate(-ngl);
  ctx.beginPath();
  ctx.moveTo(-W, 0);
  ctx.lineTo(-25, 0);
  ctx.lineTo(-10, 2.25 * dip);
  ctx.lineTo(-22, 2.0 * dip);
  ctx.lineTo(-3, 5.0 * dip);
  ctx.lineTo(8, 2.5 * dip);
  ctx.lineTo(18, 2.75 * dip);
  ctx.lineTo(25, 0);
  ctx.lineTo(W, 0);
  ctx.lineTo(W, -H);
  ctx.lineTo(-W, -H);
  ctx.closePath();
  ctx.fill();

  ctx.fillText('Jilly Bean - Strikingly Dippy', -200, 0.5 * H); 

  ngl = Math.PI * (z + 180.0) / 180.0;
  var offx = W * (0.1 + 0.8 * y / 180); // switched x, y
  var offy = H * (0.1 + 0.8 * x / 180);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.translate(offx, offy);
  ctx.rotate(-ngl);
  ctx.beginPath();
  ctx.moveTo(0, 50);
  ctx.lineTo(-20, -50);
  ctx.lineTo(20, -50);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  */ 
}

var onSuccess = function(position) {
  ////////////////////////////////////////////////////////////////
  latitude = position.coords.latitude;
  longitude = position.coords.longitude;
  altitude = position.coords.altitude;
  /*alert('Latitude: ' + latitude + '\n' +
        'Longitude: ' + longitude + '\n' +
        'Altitude: ' + altitude);*/
        /*'Accuracy: '          + position.coords.accuracy          + '\n' +
        'Altitude Accuracy: ' + position.coords.altitudeAccuracy  + '\n' +
        'Heading: '           + position.coords.heading           + '\n' +
        'Speed: '             + position.coords.speed             + '\n' +
        'Timestamp: '         + position.timestamp                + '\n');*/
};

function onError(error) {
  ////////////////////////////////////////////////////////////////
    alert('code: '    + error.code    + '\n' +
          'message: ' + error.message + '\n');
}

function onDeviceReady() {
  ////////////////////////////////////////////////////////////////
  console.log("navigator.geolocation works well");
  /*navigator.geolocation.getCurrentPosition(onSuccess, onError, 
                {maximumAge: 60000, timeout: 5000, enableHighAccuracy: true});*/
  // getCurrentPosition() doesn't seem to work with GPS on android!
  var watchId = navigator.geolocation.watchPosition(onSuccess, onError, 
                {maximumAge: 60000, timeout: 10000, enableHighAccuracy: true});
}

window.addEventListener('deviceorientation', handleOrientation);
document.addEventListener('deviceready', onDeviceReady, false);
//</script>
