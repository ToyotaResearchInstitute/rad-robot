const port = 9001;
const ws =
    new window.WebSocket('ws://' + window.location.hostname + ':' + port);
ws.binaryType = 'arraybuffer';

document.addEventListener("DOMContentLoaded", function(event) {

  const munpack = msgpack5().decode;
  var cur = {};

  // Camera and map images
  var img_video1 = document.getElementById('video1');
  var img_video2 = document.getElementById('video2');
  var img_video3 = document.getElementById('video3');

  ws.onmessage = (e) => {
    // console.log(e.data);
    const msg = munpack(new Uint8Array(e.data));
    // console.log(msg);
    Object.assign(cur, msg);
    // console.log(cur);

  };

  const draw = (timestamp) => {
    window.requestAnimationFrame(draw);
    const t = Date.now();
    const video1 = cur.video1 || cur.video0;
    if (video1) {
      cur.video1 = false;
      const blobJ = new Blob([ video1.jpg ], {'type' : 'image/jpeg'});
      window.URL.revokeObjectURL(img_video1.src);
      img_video1.src = window.URL.createObjectURL(blobJ);
      // img_camera.onload = (e) => { console.log("done") };
      }
    const video2 = cur.video2;
    if (video2) {
      cur.video2 = false;
      const blobJ = new Blob([ video2.jpg ], {'type' : 'image/jpeg'});
      window.URL.revokeObjectURL(img_video2.src);
      img_video2.src = window.URL.createObjectURL(blobJ);
      // img_camera.onload = (e) => { console.log("done") };
      }
    const video3 = cur.video3;
    if (video3) {
      cur.video3 = false;
      const blobJ = new Blob([ video3.jpg ], {'type' : 'image/jpeg'});
      window.URL.revokeObjectURL(img_video3.src);
      img_video3.src = window.URL.createObjectURL(blobJ);
      // img_camera.onload = (e) => { console.log("done") };
    }
  };

  draw();

});
