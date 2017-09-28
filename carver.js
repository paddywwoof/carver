//<script type="javascript">
var bell;// = document.querySelector('.bell');
var horn;// = document.querySelector('.horn');
var diptxt = document.querySelector('.diptxt');
var logtxt = document.querySelector('.logtxt');
diptxt.innerHTML = "+++";
logtxt.innerHTML = "---";

var xrs = [], yrs = [], zrs = [];
var zcrs = [], zsrs = []; // hold as cos and sin to avoid jump 359 -> 0
var xs = [], ys = [], zs = [];
var yaw = [];
var xadj = 0.0, yadj = 0.0, zadj = 0.0; // angle offsets
var gxadj = 0.0, gyadj = 0.0; // g component offsets

var q; // quaternion position of phone
var h; // unit vector horizontal in plane of phone
var latitude; var longitude; var altitude;
var last_tm = 0;
var WAIT = 100;
var logOn = false;
var nAlert = 0;
var last_play = 0;

function logReading() {
  ////////////////////////////////////////////////////////////////
  logOn = !logOn;
  if (logOn) {
    logtxt.innerHTML = "";
  }
}

function calibrate() {
  ////////////////////////////////////////////////////////////////
  //xadj = medianMean(xrs, 0.0);
  //yadj = medianMean(yrs, 0.0);
  //zadj = Math.atan2(medianMean(zsrs, -1.0), medianMean(zcrs, 0.0)) + 0.5 * Math.PI;
  //gxadj += g[0];
  //gyadj += g[1];
  horn.play();
}

function medianMean(arr, val) {
  // return median of values /////////////////////////////////////
  var NUM = 15;
  var END = 2;
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

function tripleProduct(a, b, c) {
  // triple product (a X b) . c //////////////////////////////////
  return a[1] * b[2] * c[0] - a[2] * b[1] * c[0] + 
         a[2] * b[0] * c[1] - a[0] * b[2] * c[1] +
         a[0] * b[1] * c[2] - a[1] * b[0] * c[2];
}

function normalize(a) {
  // normalize an nD vector //////////////////////////////////////
  var mag = Math.sqrt(a.reduce(function(sum, cur) {
                                  return sum + cur * cur;}, 0.0));
  return a.map(function(x) {return x / mag;});
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

function taitBryanToQuat(x, y, z) {
  // return quaternion as vec4 using Tait Bryan Z,X',Y'' x,y,z in degrees
  ////////////////////////////////////////////////////////////////
  var radfact = Math.PI / 360.0; // to radians and x 0.5
  x *= radfact;
  y *= radfact;
  z *= radfact;
  sx = Math.sin(x);
  cx = Math.cos(x);
  sy = Math.sin(y);
  cy = Math.cos(y);
  sz = Math.sin(z);
  cz = Math.cos(z);
  return [cx * cy * cz - sx * sy * sz,
          sx * cy * cz - cx * sy * sz,
          cx * sy * cz + sx * cy * sz,
          cx * cy * sz + sx * sy * cz];
}

function divQuat(p, q) {
  // return quaternion pq^-1 i.e. p divided by q. The result is a
  // quaternion representing the rotation FROM q TO p
  ////////////////////////////////////////////////////////////////
  var normSq = q.reduce(function(sum, cur) {return sum + cur * cur;}, 0.0);

  if (normSq === 0) {
    return [0.0, 0.0, 0.0, 0.0];
  }

  normSq = 1 / normSq;

  return [(p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + p[3] * q[3]) * normSq,
          (p[1] * q[0] - p[0] * q[1] - p[2] * q[3] + p[3] * q[2]) * normSq,
          (p[2] * q[0] - p[0] * q[2] - p[3] * q[1] + p[1] * q[3]) * normSq,
          (p[3] * q[0] - p[0] * q[3] - p[1] * q[2] + p[2] * q[1]) * normSq];
}

function axisAngle(q_in) {
  // return vec4 representing angle in radians then x,y,z vector as axis
  ////////////////////////////////////////////////////////////////
  var q = normalize(q_in); // leave original array intact
  var angle = 2.0 * Math.acos(q[0]);
  var axis = normalize(q.slice(1)); // just the x,y,z components
  return [angle, axis[0], axis[1], axis[2]];
}

function handleMotion(event) {
  // rotation accelerometer and magnetometer changes /////////////
  ////////////////////////////////////////////////////////////////
  //var rx = medianMean(xrs, event.rotationRate.beta);
  //var ry = medianMean(yrs, event.rotationRate.gamma);
  //var rz = medianMean(zrs, event.rotationRate.alpha);
  var x = medianMean(xs, event.accelerationIncludingGravity.x);
  var y = medianMean(ys, event.accelerationIncludingGravity.y);
  var z = medianMean(zs, event.accelerationIncludingGravity.z);
  q = taitBryanToQuat(0.01, 0.02, 0.03);//rx, ry, rz);
  alert(" acc " + x + " " + y + " " + z);
  var d = new Date();
  var tm = d.getTime();
  if (tm > (last_tm + WAIT)) {
    var ax_a = axisAngle(q);
    var av_yaw = medianMean(yaw, 1000.0 * ax_a[0] * dot([x, y, z], [ax_a[3], ax_a[1], ax_a[2]]) / (tm - last_tm));
    diptxt.innerHTML = ">> yaw: " + av_yaw.toFixed(2);
    if (av_yaw > 0.66 && tm > (last_play + 200)) {
      //bell.volume = av_yaw * 0.25;
      bell.play();
      last_play = tm;
    } else if (av_yaw < -0.66 && tm > (last_play + 200)) {
      //horn.volume = -av_yaw * 0.25;
      horn.play();
      last_play = tm;
    }
    if (logOn) {
      logtxt.innerHTML += "\nrot=" + av_yaw;
    }
    last_tm = tm;
  }
}

function getPhoneGapPath() {
  ////////////////////////////////////////////////////////////////
    var path = window.location.pathname;
    path = path.substr( path, path.length - 10 );
    return 'file://' + path;
}

function onDeviceReady() {
  ////////////////////////////////////////////////////////////////
  bell = new Media(getPhoneGapPath() + "bell.ogg");
  horn = new Media(getPhoneGapPath() + "horn.ogg");
  //console.log(Media);
  //alert('Media ready');
  bell.play();
}

window.addEventListener('devicemotion', handleMotion);
document.addEventListener('deviceready', onDeviceReady, false);
//</script>
