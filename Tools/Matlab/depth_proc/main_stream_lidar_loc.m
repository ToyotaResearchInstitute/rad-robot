clear all;
close all;

datetime.setDefaultFormats('defaultdate','yyyy-MM-dd');

% 1 second timeout
%s_depth = zmq('subscribe', 'tcp', '192.168.123.246', 43346);
%s_color = zmq('subscribe', 'tcp', '192.168.123.246', 43347);
%s_mesh = zmq('subscribe', 'tcp', '192.168.123.246', 43344);
s_mesh = zmq( 'subscribe', 'ipc', 'mesh0' );
addpath('./util');

count = 0;
while 1
    idx = zmq('poll',1000);  % assume only one channel
    if isempty(idx)
        disp('empty!');
       % return;
    end
    for s = 1:numel(idx)
        s_idx = idx(s);
        [data, has_more] = zmq('receive', s_idx);
        % Get the metadata
        [metadata,offset] = msgpack('unpack', data);
        if has_more, [raw, has_more] = zmq('receive', s_idx); end
        if strcmp(char(metadata.id), 'mesh0')          
            if (mod(count,2) == 0) 
                
                metadata.dims = metadata.dim;
                metadata.flag = 1;
                raw = reshape(typecast(raw, 'single'), [metadata.dim(2), metadata.dim(1)]);

                [ Planes ] = detectPlaneInstances_lidar_loc( raw, 4, metadata);    
                pose = localizeCorner_v4L(Planes,metadata)
               
            end
           count = count + 1;
        end
    end
    drawnow;
end