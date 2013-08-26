function ret = robotbody()
  global BODY POSE LIDAR SLAM
  % simple robot visualization code 
  % SJ 2013

  %Joint positions
  OFFSET_SHOULDER = [0,0.219,0.144];
  UARM_LENGTH = 0.246;
  ELBOW_OFFSET_X = 0.030;
  LARM_LENGTH = 0.242;
  OFFSET_HAND = [0.113 0.053 0];

  BODY=[];
  BODY.basesize = [0.40 0.40 0.20];
  BODY.lbodysize = [0.10 0.10 0.70];
  BODY.ubodysize = [0.20 0.40 0.180];
  BODY.shouldersize = [0.08 0.14 0.14];
  BODY.uarmsize = [0.25 0.08 0.08];
  BODY.larmsize = [0.25 0.08 0.08];
  BODY.wristsize = [0.10 0.08 0.08];
  BODY.fingersize = [0.08 0.03 0.03];
  BODY.headsize = [0.15 0.10 0.15];

  BODY.waistoffset = [0 0 0.35];
  BODY.neckoffset = [0 0 0.40];
  BODY.lshoulderoffset = [0 0.259 0.144];
  BODY.rshoulderoffset = [0 -0.259 0.144];
  BODY.uarmoffset = [0.246 0 0.03];
  BODY.larmoffset = [0.242 0 -0.03];
  BODY.lfingeroffset = [0.08 -0.03 0];
  BODY.rfingeroffset = [0.08 0.03 0];
  BODY.finger2offset = [0.08 0.0 0];

  BODY.basecom = [0 0 -0.35];
  BODY.ubodycom = [0 0 0.09];
  BODY.headcom = [0 0 0];

  BODY.uarmcom = [0.12 0 0];
  BODY.larmcom = [0.12 0 -0.03];
  BODY.wristcom = [0.05 0 0];

  BODY.fingercom = [0.05 0 0];

  BODY.neckangle=[0,0];
  BODY.waistangle=[0,0];
  BODY.larmangle=[0,0,0,0,0,0];
  BODY.rarmangle=[0,0,0,0,0,0];
  BODY.lfingerangle = 0;
  BODY.rfingerangle = 1;

  BODY.verts=zeros(200,3);
  BODY.faces=zeros(200,3);
  BODY.vert_num = 0;
  BODY.face_num = 0;

  BODY.viewpoint = 1;

  BODY.update_angles = @update_angles;
  BODY.update = @update;
  BODY.init = @init;
  BODY.set_viewpoint = @set_viewpoint;

  POSE = [];
  POSE.pose = [ 0 0 0];
  POSE.pose_odom = [ 0 0 0];
  POSE.pose_slam = [ 0 0 0];
  POSE.battery = 0;

  function set_viewpoint(h,~,flags)
    BODY.viewpoint = flags;
    calculate_transforms();
    plot_parts();
  end

  function update_viewpoint()
    flags = BODY.viewpoint;
    pose = POSE.pose_slam;
%    robot_pos = [POSE.pose(1) POSE.pose(2) 1];
  
    robot_pos = [pose(1) pose(2) 1];
    pose_dir = [cos(pose(3)) sin(pose(3))];


    cam_dist1 = 20;

    if flags==1 

      set(BODY.p,'CameraTarget',robot_pos+ 2.5*[pose_dir -0.5]);
      set(BODY.p,'CameraPosition',...
				robot_pos +2.5*[pose_dir -0.5]+ 20*[-cos(pose(3)) -sin(pose(3)) 0.7]);
      set(BODY.p,'XLim',[-3 3]+pose(1));
      set(BODY.p,'YLim',[-3 3]+pose(2));
      set(BODY.p,'ZLim',[-1 2]);
      camva(BODY.p,6.6)


    elseif flags==2 
      set(BODY.p,'CameraTarget',robot_pos + 2*[pose_dir 0] );
      set(BODY.p,'CameraPosition',robot_pos + 2*[pose_dir 0] + 18*[-1 -1 1]);
      set(BODY.p,'XLim',[-3 3]+pose(1)+2.5*pose_dir(1));
      set(BODY.p,'YLim',[-3 3]+pose(2)+2.5*pose_dir(2));
      set(BODY.p,'ZLim',[-1 2]);
      camva(BODY.p,6.6)


    elseif flags==3 %top view
      set(BODY.p,'CameraTarget',robot_pos+ 1.2*[pose_dir 0]);
      set(BODY.p,'CameraPosition',...
				robot_pos -2*[pose_dir 0]+[0 0 20]);
      set(BODY.p,'XLim',[-2.2 2.2]+pose(1));
      set(BODY.p,'YLim',[-2.2 2.2]+pose(2));
      set(BODY.p,'ZLim',[-1 2]);
      camva(BODY.p,6.6)


    elseif flags==4 %fist person view from head
%{
      set(BODY.p,'CameraTarget',robot_pos+ 1*[pose_dir 0]);
      set(BODY.p,'CameraPosition',...
				robot_pos + 2*[-cos(pose(3)) -sin(pose(3)) 5]);
      set(BODY.p,'XLim',[-0.5 1.5]+pose(1));
      set(BODY.p,'YLim',[-1 1]+pose(2));
      set(BODY.p,'ZLim',[-1 2]);
%}

      camva(BODY.p,70)
      set(BODY.p,'CameraTarget',robot_pos + 2*[pose_dir 0] + [0 0 0] );
      set(BODY.p,'CameraPosition',robot_pos + 0.2*[pose_dir 0]+ [0 0 0]);
      set(BODY.p,'XLim',[-3 3]+pose(1)+2.5*pose_dir(1));
      set(BODY.p,'YLim',[-3 3]+pose(2)+2.5*pose_dir(2));
      set(BODY.p,'ZLim',[-1 2]);

    end
  end

  function plot_mesh(pos1,pos2,pos3,pos4)
    BODY.verts(BODY.vert_num+1,:)=pos1;
    BODY.verts(BODY.vert_num+2,:)=pos2;
    BODY.verts(BODY.vert_num+3,:)=pos3;
    BODY.verts(BODY.vert_num+4,:)=pos4;
    BODY.faces(BODY.face_num+1,:)=[BODY.vert_num+1 BODY.vert_num+2 BODY.vert_num+3];
    BODY.faces(BODY.face_num+2,:)=[BODY.vert_num+2 BODY.vert_num+4 BODY.vert_num+3];
    BODY.cdatas(BODY.face_num+1,:)=[0.2 0.2 0.2];
    BODY.cdatas(BODY.face_num+2,:)=[0.2 0.2 0.2];
    BODY.vert_num = BODY.vert_num + 4;
    BODY.face_num = BODY.face_num + 2;
  end

  function plot_part(siz,offset,tr)
    tr1 = getpos(tr*trans(offset+[siz(1) siz(2) siz(3)]/2));
    tr2 = getpos(tr*trans(offset+[siz(1) siz(2) -siz(3)]/2));
    tr3 = getpos(tr*trans(offset+[siz(1) -siz(2) siz(3)]/2));
    tr4 = getpos(tr*trans(offset+[siz(1) -siz(2) -siz(3)]/2));
    tr5 = getpos(tr*trans(offset+[-siz(1) siz(2) siz(3)]/2));
    tr6 = getpos(tr*trans(offset+[-siz(1) siz(2) -siz(3)]/2));
    tr7 = getpos(tr*trans(offset+[-siz(1) -siz(2) siz(3)]/2));
    tr8 = getpos(tr*trans(offset+[-siz(1) -siz(2) -siz(3)]/2));

    plot_mesh(tr1,tr2,tr3,tr4);
    plot_mesh(tr5,tr6,tr7,tr8);
    plot_mesh(tr1,tr2,tr5,tr6);
    plot_mesh(tr3,tr4,tr7,tr8);
    plot_mesh(tr1,tr3,tr5,tr7);
    plot_mesh(tr2,tr4,tr6,tr8);
  end

  function plot_parts()
    BODY.vert_num = 0;
    BODY.face_num = 0;
    plot_part(BODY.basesize, BODY.basecom, BODY.TrLBody);
    plot_part(BODY.lbodysize, [0 0 0], BODY.TrLBody);
    plot_part(BODY.ubodysize, BODY.ubodycom, BODY.TrUBody);
    plot_part(BODY.headsize, BODY.headcom, BODY.TrHead);


    plot_part(BODY.shouldersize, [0 0 0], BODY.TrLShoulder);
    plot_part(BODY.uarmsize, BODY.uarmcom, BODY.TrLUArm);
    plot_part(BODY.larmsize, BODY.larmcom, BODY.TrLLArm);
    plot_part(BODY.shouldersize, [0 0 0],  BODY.TrRShoulder);
    plot_part(BODY.uarmsize, BODY.uarmcom, BODY.TrRUArm);
    plot_part(BODY.larmsize, BODY.larmcom, BODY.TrRLArm);

    plot_part(BODY.wristsize, BODY.wristcom, BODY.TrLWrist);
    plot_part(BODY.wristsize, BODY.wristcom, BODY.TrRWrist);

    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger11);
    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger21);
    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger12);
    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrLFinger22);

    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger11);
    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger21);
    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger12);
    plot_part(BODY.fingersize, BODY.fingercom, BODY.TrRFinger22);

    set(BODY.h,'Vertices',BODY.verts(1:BODY.vert_num,:));
    set(BODY.h,'Faces',BODY.faces(1:BODY.face_num,:));
    set(BODY.h,'FaceVertexCData',BODY.cdatas);

    update_viewpoint();

  end


  function calculate_transforms()
%		pose = POSE.pose;
		pose = POSE.pose_slam;

    lfingerangle1 = pi/3 - pi/6*BODY.lfingerangle;
    lfingerangle2 = - pi/3*BODY.lfingerangle;

    rfingerangle1 = pi/3 - pi/6*BODY.rfingerangle;
    rfingerangle2 = - pi/3*BODY.rfingerangle;

    BODY.TrLBody = eye(4)*trans([pose(1) pose(2) -0.35])*rotZ(pose(3));

    BODY.TrUBody = BODY.TrLBody*trans(BODY.waistoffset)*...
		rotZ(BODY.waistangle(1))*rotY(BODY.waistangle(2));
    BODY.TrHead = BODY.TrUBody*trans(BODY.neckoffset)*...
		rotZ(BODY.neckangle(1))*rotY(BODY.neckangle(2));


    BODY.TrLShoulder = BODY.TrUBody*trans(BODY.lshoulderoffset)*...
		rotY(BODY.larmangle(1))*rotZ(BODY.larmangle(2)) ;
    BODY.TrLUArm = BODY.TrLShoulder*rotX(BODY.larmangle(3));
    BODY.TrLLArm = BODY.TrLUArm*trans(BODY.uarmoffset)*...
		rotY(BODY.larmangle(4));
    BODY.TrLWrist = BODY.TrLLArm*trans(BODY.larmoffset)*...
			rotX(BODY.larmangle(5))*rotZ(BODY.larmangle(6));
    BODY.TrLFinger11 = BODY.TrLWrist*trans(BODY.lfingeroffset)*...
			rotZ(lfingerangle1-pi/4);
    BODY.TrLFinger12 = BODY.TrLFinger11*trans(BODY.finger2offset)*...
			rotZ(lfingerangle2);
    BODY.TrLFinger21 = BODY.TrLWrist*trans(BODY.lfingeroffset)*...
			rotZ(-lfingerangle1-pi/4);
    BODY.TrLFinger22 = BODY.TrLFinger21*trans(BODY.finger2offset)*...
			rotZ(-lfingerangle2);

    BODY.TrRShoulder = BODY.TrUBody*trans(BODY.rshoulderoffset)*...
		rotY(BODY.rarmangle(1))*rotZ(BODY.rarmangle(2)) ;
    BODY.TrRUArm = BODY.TrRShoulder*rotX(BODY.rarmangle(3));
    BODY.TrRLArm = BODY.TrRUArm*trans(BODY.uarmoffset)*...
		rotY(BODY.rarmangle(4));
    BODY.TrRWrist = BODY.TrRLArm*trans(BODY.larmoffset)*...
			rotX(BODY.rarmangle(5))*rotZ(BODY.rarmangle(6));
    BODY.TrRFinger11 = BODY.TrRWrist*trans(BODY.rfingeroffset)*...
			rotZ(rfingerangle1+ pi/4);
    BODY.TrRFinger21 = BODY.TrRWrist*trans(BODY.rfingeroffset)*...
			rotZ(-rfingerangle1+pi/4);
    BODY.TrRFinger12 = BODY.TrRFinger11*trans(BODY.finger2offset)*...
			rotZ(rfingerangle2);
    BODY.TrRFinger22 = BODY.TrRFinger21*trans(BODY.finger2offset)*...
			rotZ(-rfingerangle2);

  end

  function init(a)
    axes(a);
    BODY.p = a;
    BODY.h = patch('FaceColor','flat','EdgeColor','none',...
     'AmbientStrength',0.4,'SpecularStrength',0.9 );
    light('Position',[0 3 3]);
	  set(a,'CameraViewAngleMode','manual');
    set(a,'DataAspectRatioMode','manual')

    set(BODY.p,'XLim',[-2 2]);
    set(BODY.p,'YLim',[-2 2]);
    set(BODY.p,'ZLim',[-1 2]);
    set(a,'XGrid','on')
    set(a,'YGrid','on')

    update_angles([0 0],[0 0],[0 0 0 0 0 0],[0 0 0 0 0 0],[0 0]);
  end

  function setup_controls(b1,b2,b3,b4)

  end

  function update_angles(waistangle, neckangle, larmangle, rarmangle, grippers)
    BODY.waistangle=double(waistangle);
    BODY.neckangle=double(neckangle);
    BODY.larmangle=double(larmangle);
    BODY.rarmangle=double(rarmangle);

    fingerangle = double(grippers);
    BODY.lfingerangle = fingerangle(1);
    BODY.rfingerangle = fingerangle(2);

    calculate_transforms();
    plot_parts();
  end

  function [ nBytes ]= update(fd)
    nBytes = 0;
    while udp_recv('getQueueSize',fd) > 0
      udp_data = udp_recv('receive',fd);
      nBytes = nBytes + numel(udp_data);
    end
    data = msgpack('unpack',udp_data);
    update_angles(clean_cell(data.waistangle)...
                ,clean_cell(data.neckangle)...
                ,clean_cell(data.larmangle)...
                ,clean_cell(data.rarmangle)...
								,clean_cell(data.grippers) );
    POSE.data = data;
		POSE.pose_odom = double(clean_cell(data.pose_odom));
		POSE.pose_slam = double(clean_cell(data.pose_slam));
		POSE.pose = double(clean_cell(data.pose));
    POSE.battery = double(data.battery);

    SLAM.update_pose(POSE.pose,POSE.pose_slam);

  end

  ret= BODY;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transform functions


  function ret = rotX(angle)
    ca=cos(angle); sa=sin(angle);
    ret = [1 0 0 0;0 ca -sa 0;0 sa ca 0; 0 0 0 1];
  end

  function ret = rotY(angle)
    ca=cos(angle); sa=sin(angle);
    ret = [ca 0 sa 0; 0 1 0 0; -sa 0 ca 0; 0 0 0 1];
  end

  function ret = rotZ(angle)
    ca=cos(angle); sa=sin(angle);
    ret = [ca -sa 0 0;sa ca 0 0; 0 0 1 0; 0 0 0 1];
  end

  function ret = trans(pos)
    ret = [1 0 0 pos(1);0 1 0 pos(2); 0 0 1 pos(3); 0 0 0 1];
  end

  function ret = getpos(tr)
    ret = [tr(1,4) tr(2,4) tr(3,4)];
  end

end


