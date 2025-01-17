%addpath(genpath('.'));
addpath(genpath('../'));
clf
clear all

a=shm('mcmStatus01thor');
a2=shm('mcmWalk01thor');

t=[];
zmp_x=[];
com_x=[];
vel_x=[];

zmp_y=[];
com_y=[];
vel_y=[];

f_handle_1 = subplot(2,1,1);
p_handle_1 = plot(0,0,'r',0,0,'g',0,0,'b');
xlabel('X Axis')
legend('COM','ZMP','Walk Velocity')

f_handle_2 = subplot(2,1,2);
p_handle_2 = plot(0,0,'r',0,0,'g', 0,0,'b');
xlabel('Y Axis')

legend('COM','ZMP','Walk Velocity')

t_dur = 3.0;
x_mag = 0.3;
y_mag = 0.3;

t_old = 0;
t_nonupdate_count = 0;
should_clear = false;

while 1 
  zmp_current = a.get_uZMP();
  com_current = a.get_uTorso();
  t_current = a.get_t();

  if t_current==t_old 
  	t_nonupdate_count = t_nonupdate_count+1;
  	if t_nonupdate_count>2000 %Auto reset after 5s
  		should_clear = true;  		
	  end
  else
	  t_nonupdate_count = 0;
	  if should_clear
	  	should_clear=false;

	  	t=[];
  		zmp_x=[];
  		com_x=[];
  		vel_x=[];
  		zmp_y=[];
  		com_y=[];
  		vel_y=[];

    end
  end

 
  if t_current~=t_old 
    t_old = t_current;

    walk_vel = a2.get_vel();

    zmp_x=[zmp_x zmp_current(1)];
    com_x=[com_x com_current(1)];
    vel_x=[vel_x walk_vel(1)];

    zmp_y=[zmp_y zmp_current(2)];
    com_y=[com_y com_current(2)];
    vel_y=[vel_y walk_vel(2)];
    t=[t t_current];

    set(p_handle_1(1),'XData',t);
    set(p_handle_1(1),'YData',com_x);
    set(p_handle_1(2),'XData',t);
    set(p_handle_1(2),'YData',zmp_x);
    set(p_handle_1(3),'XData',t);
    set(p_handle_1(3),'YData',vel_x);

    set(p_handle_2(1),'XData',t);
    set(p_handle_2(1),'YData',com_y);
    set(p_handle_2(2),'XData',t);
    set(p_handle_2(2),'YData',zmp_y);
    set(p_handle_2(3),'XData',t);
    set(p_handle_2(3),'YData',vel_y);

    xlim(f_handle_1,[t_current-t_dur t_current+t_dur])
    ylim(f_handle_1,[-x_mag+com_current(1) x_mag+com_current(1)])

    xlim(f_handle_2,[t_current-t_dur t_current+t_dur])
    ylim(f_handle_2,[-y_mag+com_current(2) y_mag+com_current(2)])
    drawnow;
  end
      pause(0.005);
end