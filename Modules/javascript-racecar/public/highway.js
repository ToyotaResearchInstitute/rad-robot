document.addEventListener("DOMContentLoaded", function(event) {
  const DEG_PER_RAD = 180 / Math.PI;

  const reference_vehicle = "tri1";
  // From where to draw everything
  let frame_of_reference = false;

  ///////////////////////////
  // Identifying the SVG size
  const environment_div = document.getElementById("topdown");
  const environment_svg = document.getElementById("environment");
  var viewBox = environment_svg.getAttribute("viewBox").split(" ");
  var X_SVG_MIN = parseFloat(viewBox[0]);
  var Y_SVG_MIN = parseFloat(viewBox[1]);
  var X_SVG_SZ = parseFloat(viewBox[2]);
  var Y_SVG_SZ = parseFloat(viewBox[3]);

  // Sizing the canvas
  const environment_canvas = document.getElementById("stage");
  var environment_ctx = false;
  var X_CANVAS_SZ, Y_CANVAS_SZ;
  const X_CANVAS_MIN = 0;
  const Y_CANVAS_MIN = 0;

  const svg2canvas_sz = s => {
    return [(s[0] * X_CANVAS_SZ) / X_SVG_SZ, (s[1] * Y_CANVAS_SZ) / Y_SVG_SZ];
  };
  const svg2canvas = p => {
    return [
      (X_CANVAS_SZ * (p[0] - X_SVG_MIN)) / X_SVG_SZ + X_CANVAS_MIN,
      (Y_CANVAS_SZ * (p[1] - Y_SVG_MIN)) / Y_SVG_SZ + Y_CANVAS_MIN,
      p[2]
    ];
  };
  // const coord2svg = (p) => { return [ p[0], -p[1], -p[2] ]; };
  // Flip X and Y
  const coord2svg = p => {
    return [p[1], p[0], Math.PI / 2 - p[2]];
  };
  // End of dimensions
  ///////////////////////////

  ///////////////////
  // Process messages, and hold on to the current values
  let cur = {};
  let rendered_message = true;
  // Utilize parsing library
  const munpack = msgpack5().decode;
  //
  const port = 9001;
  const ws = new window.WebSocket(
    "ws://" + window.location.hostname + ":" + port
  );
  ws.binaryType = "arraybuffer";
  ws.onmessage = e => {
    let msg = munpack(new Uint8Array(e.data));
    Object.assign(cur, msg);
    // console.log(msg);
    rendered_message = false;
  };
  // Process messages
  ///////////////////

  ///////////////////////////
  // Animation loop: Draw items
  // Update mapping of function -> timestamp
  let visualizers = new Map();
  const animate = () => {
    // Call this function, again
    requestAnimationFrame(animate);
    // No new messages, so simply return
    if (rendered_message) {
      return;
    }
    // const t_render = Date.now();
    // Draw the current items
    // Call the function, with optional previous information, so that we do not render too quickly
    visualizers.forEach((info_prev, fn) => {
      const info = fn(cur, info_prev);
      visualizers.set(fn, info);
    });
    rendered_message = true;
  };
  animate();
  // Recompute upon resize
  window.addEventListener("resize", () => {
    rendered_message = false;
    environment_ctx = false;
  });
  ///////////////////////////

  // Handle changes in the viewer boundaries
  const update_view = (msg, info_previous) => {
    const planner = msg.planner;
    if (planner && planner.viewBox) {
      const viewBoxNew = planner.viewBox;
      const viewbox_changed = viewBoxNew.reduce((eq, v, i) => {
        return eq || v !== viewBox[i];
      }, false);
      if (viewbox_changed) {
        // Must flip the coordinates...
        X_SVG_MIN = parseFloat(viewBoxNew[1]);
        Y_SVG_MIN = parseFloat(viewBoxNew[0]);
        X_SVG_SZ = parseFloat(viewBoxNew[3]);
        Y_SVG_SZ = parseFloat(viewBoxNew[2]);
        environment_svg.setAttribute(
          "viewBox",
          X_SVG_MIN + " " + Y_SVG_MIN + " " + X_SVG_SZ + " " + Y_SVG_SZ
        );
        environment_ctx = false;
      }
    }
    // Check our sizes
    if (!environment_ctx) {
      const rect = environment_svg.getBoundingClientRect();
      X_CANVAS_SZ = environment_canvas.width = rect.width;
      Y_CANVAS_SZ = environment_canvas.height = rect.height;
      environment_ctx = environment_canvas.getContext("2d");
    }
    return true;
  };
  visualizers.set(update_view, false);

  const update_road = (msg, info_previous) => {
    const planner = msg.planner;
    if (!planner) {
      return;
    }

    if (planner.paths) {
      const lanes = planner.paths;
      const pt_to_pair = (coord, i) => {
        return coord2svg(coord)
          .slice(0, 2)
          .join();
      };
      // Grab the SVG of each lane
      var lanes_els = environment_svg.getElementsByClassName("lane");
      // Iterate the names of the lanes
      Object.keys(lanes).forEach((name, ilane) => {
        const l = lanes[name];
        const points = l["points"].map(pt_to_pair).join(" ");
        const lane_id = "lane_" + name;
        var el = lanes_els.namedItem(lane_id);
        if (!el) {
          el = document.createElementNS(
            "http://www.w3.org/2000/svg",
            "polyline"
          );
          el.setAttributeNS(null, "id", lane_id);
          el.setAttributeNS(null, "class", "lane");
          el.style.fill = "none";
          el.style.stroke = "#0F0";
          el.style.strokeWidth = "0.1";
          el.style.opacity = "0.3";
          // el.setAttributeNS(null, 'marker-start', 'url(#arrow)');
          el.setAttributeNS(null, "marker-end", "url(#marker-arrow)");
          if (name.startsWith("turn_")) {
            el.setAttributeNS(null, "marker-mid", "url(#marker-dot)");
          }
          environment_svg.appendChild(el);
        }
        el.setAttributeNS(null, "points", points);
      });
    } // end of checking for lanes

    // Try highways
    if (planner.highways) {
      console.log(planner.highways);
    }
  };
  // Add to the processor
  visualizers.set(update_road, false);

  const update_poses = (msg, info_previous) => {
    const vehicles = msg.vicon;
    if (!vehicles) {
      return;
    }
    // Check if we have seen this frame, before
    const frame = vehicles.frame;
    if (frame === info_previous) {
      return;
    }
    delete vehicles.frame;

    const vicon2pose = p => {
      return [p.translation[0] / 1e3, p.translation[1] / 1e3, p.rotation[2]];
    };

    if (vehicles["tri1"]) {
      frame_of_reference = vicon2pose(vehicles["tri1"]);
    }

    // SVG
    Object.keys(vehicles).forEach((vehicle_name, ivehicle) => {
      const vehicle = vehicles[vehicle_name];
      const vehicle_el_id = "vehicle_" + vehicle_name;
      let vehicle_el = document.getElementById(vehicle_el_id);
      if (!vehicle_el) {
        vehicle_el = document.createElementNS(
          "http://www.w3.org/2000/svg",
          "use"
        );
        vehicle_el.setAttributeNS(
          "http://www.w3.org/1999/xlink",
          "xlink:href",
          // "basevehicle"
          "#automobile"
        );
        vehicle_el.setAttributeNS(null, "id", vehicle_el_id);
        vehicle_el.setAttributeNS(null, "class", "vehicle");
        // If a certain vehicle
        // if (vehicle_name == reference_vehicle) {
        //   vehicle_el.setAttribute("fill", "brown");
        // }
        environment_svg.appendChild(vehicle_el);
      }
      // Update the properties
      let pose = vicon2pose(vehicle);
      if (frame_of_reference) {
        // NOTE: For highway, do not use angles, so OK
        pose[0] = pose[0] - frame_of_reference[0] + 1;
      }
      const coord_svg = coord2svg(pose);
      vehicle_el.setAttributeNS(null, "x", coord_svg[0]);
      vehicle_el.setAttributeNS(null, "y", coord_svg[1]);
      const tfm_vel =
        "rotate(" +
        [coord_svg[2] * DEG_PER_RAD, coord_svg[0], coord_svg[1]].join() +
        ")";
      vehicle_el.setAttributeNS(null, "transform", tfm_vel);
    });
    // THREE.js
    // while (vehicles.length > veh_boxes.length) {
    //   const veh = veh_mesh.clone();
    //   scene.add(veh);
    //   veh_boxes.push(veh);
    // }
    // while (vehicles.length < veh_boxes.length) {
    //   scene.remove(veh_boxes.pop());
    // }
    // vehicles.forEach((v, i) => {
    //   const veh = veh_boxes[i];
    //   veh.position.x = v[0];
    //   veh.position.y = v[1];
    //   veh.rotation.z = v[2];
    // });
    return frame;
  };
  // Add to the processor
  visualizers.set(update_poses, false);

  const update_control = (msg, info_previous) => {
    const ctrl = msg.control;
    if (!ctrl) {
      return;
    }
    const id_robot = ctrl.id;

    // Lookahead
    const p_lookahead = ctrl.p_lookahead;
    if (p_lookahead) {
      var pla_el = document.getElementById("lookahead_" + id_robot);
      if (!pla_el) {
        pla_el = document.createElementNS(
          "http://www.w3.org/2000/svg",
          "circle"
        );
        pla_el.setAttributeNS(null, "id", "lookahead_" + id_robot);
        pla_el.setAttributeNS(null, "r", 0.05);
        pla_el.setAttributeNS(null, "class", "lookahead");
        pla_el.style.fill = "red";
        pla_el.style.stroke = "none";
        environment_svg.appendChild(pla_el);
      }
      const pla = coord2svg(p_lookahead);
      pla_el.setAttributeNS(null, "cx", pla[0]);
      pla_el.setAttributeNS(null, "cy", pla[1]);
    }

    // Near
    const p_near = ctrl.p_path;
    if (p_near) {
      var pn_el = document.getElementById("near_" + id_robot);
      if (!pn_el) {
        pn_el = document.createElementNS(
          "http://www.w3.org/2000/svg",
          "circle"
        );
        pn_el.setAttributeNS(null, "id", "near_" + id_robot);
        pn_el.setAttributeNS(null, "r", 0.05);
        pn_el.setAttributeNS(null, "class", "near");
        pn_el.style.fill = "yellow";
        pn_el.style.stroke = "none";
        environment_svg.appendChild(pn_el);
      }
      const pn = coord2svg(p_near);
      const xn = pn[0] || 0;
      const yn = pn[1] || 0;
      pn_el.setAttributeNS(null, "cx", xn);
      pn_el.setAttributeNS(null, "cy", yn);
    }
    return false;
  };
  // Add to the processor
  visualizers.set(update_control, false);
});
