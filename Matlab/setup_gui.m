%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UPenn THOR Human-Robot Interface
%% Copyright 2013 Stephen McGill
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = setup_gui()
warning off;
global H_FIGURE;
global CAMERA LIDAR BODY SLAM NETMON CONTROL HMAP DEBUGMON

camera_sq_sz = .5;
buffer = 0.01;

FIRST_COL  = 0 + buffer/2;
SECOND_COL = (1-camera_sq_sz)/2 + buffer/2;
THIRD_COL  = 1-(1-camera_sq_sz)/2 + buffer/2;

FIRST_ROW = 1-camera_sq_sz/2 + buffer/2;
SECOND_ROW_CAM = 1-camera_sq_sz + buffer/2;
THIRD_ROW_CAM = 0 + buffer/2;

FIRST_COL_W = (1-camera_sq_sz)/2 - buffer;
SECOND_COL_W = camera_sq_sz - buffer;
THIRD_COL_W = (1-camera_sq_sz)/2 - buffer;

FIRST_ROW_H = camera_sq_sz/2 - buffer;
SECOND_ROW_H = camera_sq_sz/2 - buffer;
THIRD_ROW_H = 1-camera_sq_sz - buffer;

sz1 = 0.3;
sz2 = 0.76;

FIRST_ROW = 1-sz1 + buffer/2;
SECOND_ROW = 1-sz2 + buffer/2;
THIRD_ROW = 0 + buffer/2;

FIRST_ROW_H = sz1 - buffer;
SECOND_ROW_H = (sz2-sz1) - buffer;
THIRD_ROW_H = 1-(sz2) - buffer;







%% Setup the main figure
f = figure(1);
H_FIGURE = gcf;
clf;
set(H_FIGURE, 'Name', 'UPenn DRC monitor', ...
    'NumberTitle', 'off', ...
    'tag', 'Colortable', ...
    'MenuBar', 'none', ...
    'ToolBar', 'figure', ...
    'Color', [.05 .1 .05], ...
    'Colormap', gray(256), ...
    'position',[1 1 1600 900], ...
    'doublebuffer','off' );

% 	'KeyPressFcn',@KeyResponse);


CONTROL = controlbody();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

H_MESH_AXES = axes('Parent', H_FIGURE, ...
    'XTick', [], 'YTick', [], 'Units', 'Normalized', ...
    'Position', [SECOND_COL+0.05 0.17 SECOND_COL_W-0.10 0.3]);

H_DMAP_AXES = axes('Parent', H_FIGURE, ...
    'XTick', [], 'YTick', [], 'Units', 'Normalized', ...
    'Position', [THIRD_COL SECOND_ROW THIRD_COL_W SECOND_ROW_H]);

% Robot visualization

BODY = robotbody();
BODY.init(H_MESH_AXES);

LIDAR = lidarbody();
LIDAR.init(H_DMAP_AXES,H_MESH_AXES);


rb1=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'View 1', 'Units', 'Normalized', ...
    'Position', [SECOND_COL SECOND_ROW_CAM-0.06 0.05 0.04] );

rb2=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'View 2', 'Units', 'Normalized', ...
    'Position', [SECOND_COL SECOND_ROW_CAM-0.10 0.05 0.04] );

rb3=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'View 3', 'Units', 'Normalized', ...
    'Position', [SECOND_COL SECOND_ROW_CAM-0.14 0.05 0.04] );

rb4=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'View 4', 'Units', 'Normalized', ...
    'Position', [SECOND_COL SECOND_ROW_CAM-0.18 0.05 0.04] );

CONTROL.setup_robotbody_controls(rb1,rb2,rb3,rb4);


% 3d mesh controls

lb1=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'Head LIDAR', 'Units', 'Normalized', ...
    'Position', [SECOND_COL SECOND_ROW_CAM-0.26 0.05 0.04] );

lb2=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'Chest LIDAR', 'Units', 'Normalized', ...
    'Position', [SECOND_COL SECOND_ROW_CAM-0.30 0.05 0.04] );

%depth map controls

lmb1 = uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', '+', 'Units', 'Normalized', ...
    'Position', [THIRD_COL+0.00 SECOND_ROW-0.05 0.02 0.04]);

lmb2 = uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', '-', 'Units', 'Normalized', ...
    'Position', [THIRD_COL+0.02 SECOND_ROW-0.05 0.02 0.04]);

lmb3 = uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'Clear', 'Units', 'Normalized', ...
    'FontSize', 14, ...
    'Position', [THIRD_COL+0.00 SECOND_ROW-0.09 0.04 0.04]);

lmb4 = uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'Chest', 'Units', 'Normalized', ...
    'FontSize', 14, ...
    'Position', [THIRD_COL+0.00 SECOND_ROW-0.12 0.04 0.04]);
lmb5 = uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
    'String', 'Calculate', 'Units', 'Normalized', ...
    'FontSize', 14, ...
    'Position', [THIRD_COL+0.00 SECOND_ROW-0.15 0.04 0.04]);  

CONTROL.setup_lidarbody_controls(lb1,lb2,lmb1,lmb2,lmb3,lmb4,lmb5);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Head Camera
    
    HCAMERA_AXES = axes('Parent', H_FIGURE, ...
        'YDir', 'reverse','XTick', [],'YTick', [], 'Units', 'Normalized', ...
        'Position', [SECOND_COL SECOND_ROW_CAM SECOND_COL_W camera_sq_sz-buffer]);
    
    % Left hand camera
    
    LCAMERA_AXES = axes('Parent', H_FIGURE, ...
        'XTick', [], 'YTick', [], 'Units', 'Normalized', ...
        'Position', [FIRST_COL FIRST_ROW FIRST_COL_W FIRST_ROW_H]);
    
    % Right hand camera
    RCAMERA_AXES = axes('Parent', H_FIGURE, ...
        'XTick', [], 'YTick', [], 'Units', 'Normalized', ...
        'Position', [THIRD_COL FIRST_ROW THIRD_COL_W FIRST_ROW_H]);
    
    CAMERA = camerabody();
    CAMERA.init(HCAMERA_AXES,LCAMERA_AXES,RCAMERA_AXES);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % SLAM image
    H_SLAM_AXES = axes('Parent', H_FIGURE, ...
        'XDir','reverse', 'XTick', [], 'YTick', [],'Units', 'Normalized', ...
        'Position', [FIRST_COL SECOND_ROW FIRST_COL_W SECOND_ROW_H]);
    
    SLAM  = slambody();
    SLAM.init( H_SLAM_AXES);
    
    sb1=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', '+', 'Units', 'Normalized', ...
        'Position', [SECOND_COL-0.05 SECOND_ROW-0.05 0.02 0.04]);
    
    sb2=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', '-', 'Units', 'Normalized', ...
        'Position', [SECOND_COL-0.03 SECOND_ROW-0.05 0.02 0.04]);
    
    CONTROL.setup_slambody_controls(sb1,sb2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 	% HMAP image
    % 	H_HMAP_AXES = axes('Parent', H_FIGURE, ...
    % 	'XDir','reverse', 'XTick', [], 'YTick', [],'Units', 'Normalized', ...
    % 	'Position', [FIRST_COL SECOND_ROW FIRST_COL_W SECOND_ROW_H]);
    %
    %   HMAP  = hmapbody();
    %   HMAP.init( H_HMAP_AXES);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Network Status monitor
    
    H_NETWORK_AXES = axes('Parent', H_FIGURE, ...
        'XTick', [], 'YTick', [], 'Units', 'Normalized', ...
        'Position', [SECOND_COL-0.065 0.005 0.18 0.080]);
    
    H_NETWORK_TEXT = uicontrol('Style','text','Units','Normalized',...
        'Position', [SECOND_COL-0.065 0.085 0.18 0.02]);
    
    H_DEBUG_TEXT = uicontrol('Style','text','Units','Normalized',...
        'Position', [SECOND_COL-0.065+0.18 0.005 0.11 0.10]);
    
    
    
    NETMON = netmonbody();
    NETMON.init(H_NETWORK_AXES,H_NETWORK_TEXT);
    
    DEBUGMON = debugbody();
    DEBUGMON.init(H_DEBUG_TEXT);
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Body FSM control
    %%
    
    b1=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Stop', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.01 SECOND_ROW-0.06 0.18 0.06]);
    
    b2=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Navigate', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.01 SECOND_ROW-0.12 0.18 0.06] ...
        );
    b3=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Point approach', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.01 SECOND_ROW-0.18 0.18 0.06] ...
        );
    b4=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Wheel approach', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.01 SECOND_ROW-0.24 0.18 0.06] ...
        );
    CONTROL.setup_body_controls(b1,b2,b3,b4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Head FSM control
    %%
    
    h1=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Head Fixed', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [SECOND_COL+0.265 0.005 0.06 0.08]);
    
    h2=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Head Free', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [SECOND_COL+0.325 0.005 0.06 0.08]);
    
    h3=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Quick scan', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [SECOND_COL+0.385 0.005 0.085 0.08]);
    
    h4=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Slow scan', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [SECOND_COL+0.47 0.005 0.090 0.08]);
    
    CONTROL.setup_head_controls(h1,h2,h3,h4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Arm state machine control
    
    ARM_CONTROL_ROW = SECOND_ROW;
    
    
    a1=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Arm init', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.81 ARM_CONTROL_ROW-0.06 0.18 0.06]);
    
    a2=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Wheel Grab', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.81 ARM_CONTROL_ROW-0.12 0.18 0.06]);
    
    a3=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'Arm reset', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.81 ARM_CONTROL_ROW-0.18 0.18 0.06]);
    
    a4=uicontrol('Parent', H_FIGURE, 'Style', 'pushbutton', ...
        'String', 'STOP', ...
        'FontSize', 14, ...
        'Units', 'Normalized', ...
        'Position', [0.81 ARM_CONTROL_ROW-0.24 0.18 0.06]);
    
    
    CONTROL.setup_arm_controls(a1,a2,a3,a4);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
