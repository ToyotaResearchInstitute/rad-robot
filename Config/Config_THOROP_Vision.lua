assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

local head = {
  pitchMin = 0 * math.pi/180,
  pitchMax = 45 * math.pi/180,
  yawMin = -135 * math.pi/180,
  yawMax = 135 * math.pi/180,
  --TODO: Need to recalibrate position
  cameraPos = {0.034, 0.0, 0.1},
  cameraAngle = {0, 0},
  --From CoM to neck joint
  neckZ = .165 + .161,
  neckX = 0,
}

local vision = {
  colors = {
    orange = 1,
    yellow = 2,
    cyan = 4,
    field = 8,
    white = 16,
  }
  -- TODO: vision.colorBlack for obstacles
  scaleA = 2,
  scaleB = 4,
  use_white_wall = 1,
  white_wall_is_blue = 0,
  white_wall_min_count = 3000,
  white_wall_min_rate = 0.3,
  use_nonwhite_wall = 0,
  nonwhite_wall_min_area = 3000,
  nonwhite_wall_max_rate = 0.15,
  --
}

vision.ball = {
  diameter = 0.22,
  th_min_color = 50, --100
  th_min_color2 = 6,
  th_min_fill_rate = 0.35,
  th_height_max  = 0.20,
  th_ground_boundingbox = {-30,30,0,20},
  th_min_green1 = 400,
  th_min_green2 = 150,
  check_for_ground = 1,
  check_for_field = 1,
  field_margin = 2.0,
}

vision.goal = {
  th_min_color = 120,
  th_nPostB = 5,
  th_min_area = 40,
  th_min_orientation = 60*math.pi/180,
  th_min_fill_rate = 0.4,
  height_min = -0.9,
  th_aspect_ratio = {2.5,35},
  th_edge_margin = 5,
  th_bottom_boundingbox = 0.9,
  th_ground_boundingbox = {-15,15,-15,10},
  th_min_green_ratio = 0.2,
  th_min_bad_color_ratio = 0.1,
  th_goal_separation = {0.35,3.0},
  th_min_area_unknown_post = 100,
  -- The range we triangulate:
  far_goal_threshold= 4.0,
  use_centerpost = 1,
  min_crossbar_ratio = 0.6,
  check_for_ground = 1,
}

-- Black obstacle
vision.obstacle = {
  th_min_color = 300,
  th_min_area = 60,
  th_min_orientation = 60*math.pi/180,
  th_fill_rate = 0.5,
  height_min = -0.9,
  height_max = -0.5,
  th_aspect_ratio = {2.5, 6},
}

-- Line
vision.line = {
  min_white_pixel = 300,
  min_green_pixel = 5000,
  max_width = 8,
  connect_th = 1.4,
  max_gap = 1,
  -- labelB space
  min_count = 20,
  min_length = 10,
  max_height = 0.3,
  min_aspect_ratio = 2.5,
  min_angle_diff = 15,
  max_angle_diff = 85,
}

-------------
-- Cameras --
-------------
Config.camera = {}
table.insert(Config.camera,
  {
    name = 'head',
    dev = '/dev/video0',
    format = 'yuyv',
    w = 640,
    h = 360, --480,
    fps = 30,
    udp_port = 33333,
    lA_port = 33335,
    jpeg_quality = 60,
    -- Use the standard head properties
    head = head,
    -- Use the default vision parameters
    vision = vision,
    -- Run the standard RoboCup vision processing
    detection_pipeline = {
      'libVision2',
    }
    --[[
    -- Logitech C920
    lut = '308_wide_night.raw',
    focal_length = 533,
    focal_base = 640,
    auto_param = {
      {'White Balance Temperature, Auto', 0},
      {'Power Line Frequency', 0},
      {'Exposure, Auto', 1},  -- 1 for manual?
      {'Exposure, Auto Priority', 0},
    },
    param = {
      {'Brightness', 76},
      {'Contrast', 117},
      {'Saturation', 130},
      --{'Exposure (Absolute)', 90},
      {'Gain', 105},
      {'Sharpness', 0},
      {'White Balance Temperature', 2885},
      --{'Backlight Compensation', 1},
    },
    --]]
    ---[[
    --Logitech C905
    lut = '308_narrow_night.raw',
    focal_length = 554.256,
    focal_base = 640,
    auto_param = {
      {'White Balance Temperature, Auto', 0},
      {'Power Line Frequency', 0},
      {'Exposure, Auto', 1},
      {'Exposure, Auto Priority', 0},
    },
    param = {
      {'Brightness', 0},
      {'Contrast', 10},
      {'Saturation', 18},
      {'Exposure (Absolute)', 800},
      {'Gain', 255},
      {'Sharpness', 0},
      {'White Balance Temperature', 1000},
    },
    --]]
  })

--Webots use 1/2 resolution but 2x label resolution
if IS_WEBOTS then
  camera.head.resolution = {320,180}
  camera.head.x_center = 160
  camera.head.y_center = 90
  camera.head.focal_length = 554.256/2

  camera.head.focal_base = 320
	head.neckX= 0 --From CoM to neck joint

  head.cameraPos = {0.0785, 0, 0.072}

  vision.scaleA = 1
  vision.scaleB = 4
  -- Karen: smaller labelA resolution reduces vcm burden
  vision.scaleA = 2
  vision.scaleB = 2
  camera.lut_file = 'lutWebots74.raw'
end

-- Associate with the table
Config.camera = camera
Config.vision = vision
Config.head = head

return Config
