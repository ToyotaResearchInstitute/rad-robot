const port = 9001;
const ws = new window.WebSocket('ws://' + window.location.hostname + ':' + port);
ws.binaryType = 'arraybuffer';
var cur = {};
document.addEventListener("DOMContentLoaded", function(event) {

	const munpack = msgpack5().decode;

	// Create a time series
	const series_colors = [
		'red',
		'green',
		'blue'
	]

	// Plot the LIDAR points
	const container_points = document.getElementById('container_points');
	var stats;
	var camera, scene, renderer;
	var points, pointsH;
	var cyl_imu, cyl_ukf;

	// Find the canvases for IMU signal
	var canvas_rpy = document.getElementById('rpy');

	// Camera and map images
	var img_camera = document.getElementById('camera');
	var img_map = document.getElementById('map');

	// Plot the orientation
	var container_orientation = document.getElementById('container_orientation');

	// Create the chart
	var chart_rpy = new SmoothieChart({interpolation: 'step',});
	var series_rpy = [];
	for(var i=0;i<3;i++){
		const ts = new TimeSeries();
		series_rpy[i] = ts;
		chart_rpy.addTimeSeries(ts, {
			strokeStyle: series_colors[i],
			lineWidth: 3
		});
	}
	chart_rpy.streamTo(canvas_rpy, 0);

	var canvas_gyro = document.getElementById('gyro');
	var chart_gyro = new SmoothieChart({interpolation: 'step',});
	var series_gyro = [];
	for(var i=0;i<3;i++){
		const ts = new TimeSeries();
		series_gyro[i] = ts;
		chart_gyro.addTimeSeries(ts, {
			strokeStyle: series_colors[i],
			lineWidth: 2
		});
	}
	chart_gyro.streamTo(canvas_gyro, 0);

	var canvas_accel = document.getElementById('accel');
	var chart_accel = new SmoothieChart({interpolation: 'step',});
	var series_accel = [];
	for(var i=0;i<3;i++){
		const ts = new TimeSeries();
		series_accel[i] = ts;
		chart_accel.addTimeSeries(ts, {
			strokeStyle: series_colors[i],
			lineWidth: 2
		});
	}
	chart_accel.streamTo(canvas_accel, 0);

	ws.onmessage = (e) => {
		// console.log(e.data);
		const msg = munpack(new Uint8Array(e.data));
		// console.log(msg);
		Object.assign(cur, msg);
		// console.log(cur);

	};

	var drawPlot = (timestamp) => {
		window.requestAnimationFrame(drawPlot);
		// console.log("Plotting", cur);
		const t = Date.now();
		const imu = cur.inertial || cur.imu;
		if(imu) {
			series_rpy.forEach((v, i) => {
				v.append(t, imu.rpy[i] * 180 / Math.PI);
			});
			series_gyro.forEach((v, i) => {
				v.append(t, imu.gyro[i]);
			});
			series_accel.forEach((v, i) => {
				v.append(t, imu.accel[i]);
			});

			var q_ukf = new THREE.Quaternion(
				imu.q_ukf[1],
				imu.q_ukf[2],
				imu.q_ukf[3],
				imu.q_ukf[0],
			);

			// var eul_imu = new THREE.Euler(
			// 	ukf.rpy[0],
			// 	ukf.rpy[1],
			// 	ukf.rpy[2],
			// 	'XYZ'
			// );

			cyl_ukf.setRotationFromQuaternion(q_ukf);
			// cyl_imu.setRotationFromQuaternion(q_imu);
			// cyl_imu.setRotationFromQuaternion(q_gyro);
			// cyl_imu.setRotationFromEuler(eul_imu);

			cur.inertial = cur.imu = false;
		}
		const point_cloud = cur.point_cloud
		if (point_cloud) {
			cur.point_cloud = false;
			const positions = point_cloud['pts'];
			const int = point_cloud['int'];
			var colors = [];
			for (var i=0; i<positions.length;i++){
				const v = int ? (int[i] / 255) : 1;
				// const v = 1;
				colors.push(v, 1, v);
			}
			swapPoints(positions, colors);
		}
		const point_cloud_hokuyo = cur.point_cloud_hokuyo
		if (point_cloud_hokuyo) {
			cur.point_cloud_hokuyo = false;
			const pts = point_cloud_hokuyo['pts'];
			const px = pts[0], py = pts[1], pz = pts[2];
			const int = point_cloud_hokuyo['int'];
			// console.log(int)
			var colors = [], positions = [];
			for (var i=0; i<px.length;i++){
				// const v = int ? (int[i] / 2048) : 1;
				const v = int ? Math.min(1, int[i] / 1024) : 1;
				colors.push(v, v, v);
				positions.push(px[i]*1e3, py[i]*1e3, pz[i]*1e3)
			}
			swapHokuyoPoints(positions, colors);
		}

		const slam = cur.slam;
		if (slam) {
			cur.slam = false;
			const blob = new Blob([slam['png']],
				{'type': 'image/png'});
			window.URL.revokeObjectURL(img_map.src);
			img_map.src = window.URL.createObjectURL(blob);
			// img_camera.onload = (e) => { console.log("done") };
		}
		const camera = cur.camera;
		if (camera) {
			cur.camera = false;
			const blobJ = new Blob([camera['jpg']],
				{'type': 'image/jpeg'});
			window.URL.revokeObjectURL(img_camera.src);
			img_camera.src = window.URL.createObjectURL(blobJ);
			// img_camera.onload = (e) => { console.log("done") };
		}

	}

	function swapPoints(positions, colors) {
		if (!points) {
			var geometry = new THREE.BufferGeometry();
			geometry.addAttribute( 'position',
				new THREE.Float32BufferAttribute( positions, 3 ) );
			if (colors) {
				geometry.addAttribute( 'color',
					new THREE.Float32BufferAttribute( colors, 3 ) );
			}
			geometry.computeBoundingSphere();
			var material = new THREE.PointsMaterial({
				size: 15, vertexColors: THREE.VertexColors
			});
			points = new THREE.Points( geometry, material );
			scene.add(points);
		} else {
			var p = points.geometry.attributes.position.array;
			if(p.length != positions.length) {

			}

			p.set(positions)
			points.geometry.attributes.position.needsUpdate = true;
		}
	}

	function swapHokuyoPoints(positions, colors) {
		if (!pointsH) {
			var geometry = new THREE.BufferGeometry();
			geometry.addAttribute( 'position',
				new THREE.Float32BufferAttribute( positions, 3 ) );
			if (colors) {
				geometry.addAttribute( 'color',
					new THREE.Float32BufferAttribute( colors, 3 ) );
			}
			geometry.computeBoundingSphere();
			var material = new THREE.PointsMaterial({
				size: 15, vertexColors: THREE.VertexColors
			});
			pointsH = new THREE.Points( geometry, material );
			scene.add(pointsH);
		} else {
			var p = pointsH.geometry.attributes.position.array;
			if(p.length === positions.length) {
				p.set(positions)
				pointsH.geometry.attributes.position.needsUpdate = true;
			} else {
				console.log("Bad points!")
			}
		}
	}

	function init_orientation() {
		camera_orientation = new THREE.PerspectiveCamera(
			60, window.innerWidth / window.innerHeight, 5, 100 );
		// var helper = new THREE.CameraHelper( camera );
		// scene.add(helper);
		camera_orientation.up = new THREE.Vector3( 0, 0, 1 );
		camera_orientation.position.x = 10;
		camera_orientation.position.y = 10;
		camera_orientation.position.z = 10;
		camera_orientation.lookAt(new THREE.Vector3(0,0,0));
		// controls = new THREE.OrbitControls(camera);
		scene_orientation = new THREE.Scene();
		scene_orientation.background = new THREE.Color(0x808080);
		scene_orientation.add(new THREE.AxesHelper(5));
		
		// scene.background = new THREE.Color( 0x050505 );
		// scene.fog = new THREE.Fog( 0x050505, 2000, 3500 );

		// var geometry = new THREE.CylinderGeometry( 2, 3, 5, 16 );
		// var material = new THREE.MeshBasicMaterial({
		// 	color: 0x00ff00, transparent: true, opacity: 0.5} );

		// colors
		const red = new THREE.Color(1, 0, 0);
		const green = new THREE.Color(0, 1, 0);
		const blue = new THREE.Color(0, 0, 1);
		const csub = new THREE.Color(0.5, 0.5, .5);
		const colors = [red, green, blue];
		const material_ukf = new THREE.MeshBasicMaterial({
			color: 0xffffff,
			transparent: true, opacity: 0.8,
			vertexColors: THREE.FaceColors
		});
		const geometry_ukf = new THREE.BoxGeometry(4, 3, 2);
		for (var i = 0; i < 3; i++) {
			const top = colors[i].clone();
			geometry_ukf.faces[4 * i].color = colors[i];
			geometry_ukf.faces[4 * i + 1].color = colors[i];
			const btm = top.clone().add(csub);
			geometry_ukf.faces[4 * i + 2].color = btm;
			geometry_ukf.faces[4 * i + 3].color = btm;
		}

		cyl_ukf= new THREE.Mesh(geometry_ukf, material_ukf);
		eul = new THREE.Euler(
			0 * Math.PI / 4,
			0 * Math.PI / 4,
			0 * Math.PI / 4,
			'XYZ');
		cyl_ukf.setRotationFromEuler(eul);
		scene_orientation.add(cyl_ukf);

		cyl_imu = cyl_ukf.clone();
		cyl_imu.scale.set(1.1, 1.1, 1.1);
		scene_orientation.add(cyl_imu);

		renderer_orientation = new THREE.WebGLRenderer();
		renderer_orientation.setPixelRatio(window.devicePixelRatio);
		renderer_orientation.setSize( window.innerWidth/2, window.innerHeight/2);
		// renderer_orientation.setSize(
		// 	container_orientation.scrollWidth, container_orientation.scrollHeight);
		container_orientation.appendChild(renderer_orientation.domElement);
	}
	function render_orientation() {
		renderer_orientation.render(scene_orientation, camera_orientation);
	}
	function animate_orientation() {
		requestAnimationFrame(animate_orientation);
		render_orientation();
	}

	function init_points() {
		camera = new THREE.PerspectiveCamera(
			60, window.innerWidth / window.innerHeight, 5, 1000000 );
		// var helper = new THREE.CameraHelper( camera );
		// scene.add(helper);
		camera.up = new THREE.Vector3( 0, 0, 1 );
		camera.position.z = 10000; //2750;
		// controls = new THREE.OrbitControls(camera);
		scene = new THREE.Scene();
		// scene.background = new THREE.Color( 0x050505 );
		// scene.background = new THREE.Color( 0x050505 );
		// scene.fog = new THREE.Fog( 0x050505, 2000, 3500 );

		var geometry = new THREE.CylinderGeometry( 300, 300, 500, 16 );
		var material = new THREE.MeshBasicMaterial( {color: 0xffff00} );
		var cylinder = new THREE.Mesh( geometry, material );
		scene.add( cylinder );

		renderer = new THREE.WebGLRenderer();
		renderer.setPixelRatio( window.devicePixelRatio );
		renderer.setSize( window.innerWidth/2, window.innerHeight/2);
		// renderer.setSize(
		// 	container_points.scrollWidth, container_points.scrollHeight);
		container_points.appendChild( renderer.domElement );
	}

	function onWindowResize() {
		camera.aspect = window.innerWidth / window.innerHeight;
		camera.updateProjectionMatrix();
		renderer.setSize(window.innerWidth/2, window.innerHeight/2);

		renderer_orientation.setSize(
			container_points.scrollWidth, container_points.scrollHeight);
	}

	function animate_points() {
		requestAnimationFrame(animate_points);
		render();
		// stats.update();
	}

	function render() {
		var time = Date.now() * 0.001;
		// points.rotation.x = time * 0.25;
		// points.rotation.y = time * 0.5;
		renderer.render(scene, camera);
	}

	drawPlot();
	init_points();
	animate_points();
	init_orientation();
	animate_orientation();
	window.addEventListener('resize', onWindowResize, false);

});

