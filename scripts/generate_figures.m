% generate_figures.m
% Ed Kelley
% Senior thesis, 2012-2013

% 'self.step'
% 'delta_t'
% 'x_acc'
% 'y_acc'
% 'z_acc'
% 'gyr_theta_est'
% 'rotX'
% 'rotY'
% 'delta_theta'
% 'self.acc_est.x'
% 'self.acc_est.y'
% 'self.acc_est.z'
% 'self.acc_est.theta'
% 'x_vel'
% 'y_vel'
% 'z_est'
% 'magX'
% 'magY'
% 'magZ'
% 'mag_theta_est'
% 'new_x'
% 'new_y'
% 'self.vis_est.x'
% 'self.vis_est.y'
% 'self.est.x'
% 'self.est.y'
% 'self.est.theta'


function [ data_obj ] = generate_figures( filename, percentage )
	data_obj = importdata(filename, ',', 1);

	ar_filename = strcat(filename(1:end-4), '_ar.txt');
	alt_filename = strcat(filename(1:end-4), '_alt.txt');
	part_filename = strcat(filename(1:end-4), '_part.txt');
	sensor_filename = strcat(filename(1:end-4), '_sensor.txt');
	data_ar = importdata(ar_filename, ',');
	data_alt = importdata(alt_filename, ',');
	data_part = importdata(part_filename, ',');
	data_sensor = importdata(sensor_filename, ',');

	% data = data_obj.data;
	% data = data(2:end, :);
	% % acc_pos(data);
	% vis_pos(data);
	% vis_readings(data);
	% euler_method(data);
	% plot_acc(data);
	% rotation_values(data);

	% ar_pos(data_ar);
	% alt_pos(data_alt);


	% particles(data_ar, data_alt, data_part, percentage);
	% prism();
	% ultra1(data_sensor);
	% ultra2(data_sensor);
	% gyr(data_sensor);
	ar();
end



function [] = ar()
	% self.step, marker_id, self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta
	tests_mat = ['~/Dropbox/thesis_data/2013_04_28_16_53_10.txt';
	 '~/Dropbox/thesis_data/2013_04_28_16_54_03.txt';
	 '~/Dropbox/thesis_data/2013_04_28_16_54_44.txt';
	  '~/Dropbox/thesis_data/2013_04_28_16_55_27.txt';
	  '~/Dropbox/thesis_data/2013_04_28_16_57_58.txt';
	  '~/Dropbox/thesis_data/2013_04_28_16_59_03.txt'];

	tests = cellstr(tests_mat);

	percent_mat = [,];
	xvar_mat = [,];
	yvar_mat = [,];
	zvar_mat = [,];

	for i=1:6
		filename = tests{i};
		dist = (3 - (i-1)*.5)*1000;
		ar_filename = strcat(filename(1:end-4), '_ar.txt');
		data_ar = importdata(ar_filename, ',');

		prev_step = 0;
		missed = 0;
		for j=1:length(data_ar)
			step = data_ar(j, 1);
			if ((step-prev_step) > 1)
				missed = missed + 1;
			end
			prev_step = step;
		end

		percent_missed = missed/data_ar(length(data_ar), 1);
		percent_correct = (1-percent_missed)*100;
		percent_mat(i, :) = [dist, percent_correct];
		xvar_mat(i, :) = [dist, std(data_ar(:, 3))];
		yvar_mat(i, :) = [dist, std(data_ar(:, 4))];
		zvar_mat(i, :) = [dist, std(data_ar(:, 5))];
	end

	disp(percent_mat);
	disp(xvar_mat);
	disp(yvar_mat);
	disp(zvar_mat);
	create_ar_var(xvar_mat, 'X Standard Deviation (mm)', 'x');
	create_ar_var(yvar_mat, 'Y Standard Deviation (mm)', 'y');
	create_ar_var(zvar_mat, 'Z Standard Deviation (mm)', 'z');
end


function [] = create_ar_var(data, graph_title, short)
	hFig = figure;
	hold on;
	grid on;

	hTitle = title(strcat(graph_title, ' vs. Distance (mm)'));
	hXLabel = xlabel('Distance (mm)');
	hYLabel = ylabel(graph_title);
	hPlot = plot(data(:, 1), data(:, 2), '-s');

	set(hPlot,...
		'Color', [168/255, 191/255, 183/255],...
		'MarkerEdgeColor',[19/255, 37/255, 55/255],...
		'LineWidth', 3);

	set(gca,...
    'FontName', 'Helvetica');

    set([hTitle, hXLabel, hYLabel], ...
    'FontName', 'AvantGarde');

    set( hTitle, ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

	     set(gca, ...
	  'Box'         , 'off'     , ...
	  'TickDir'     , 'out'     , ...
	  'TickLength'  , [.02 .02] , ...
	  'XMinorTick'  , 'on'      , ...
	  'YMinorTick'  , 'on'      , ...
	  'ZMinorTick'  , 'on'      , ...
	  'XColor'      , [.3 .3 .3], ...
	  'YColor'      , [.3 .3 .3], ...
	  'ZColor'      , [.3 .3 .3], ...
	  'LineWidth'   , 1         );

	hold off;

	set(gcf, 'PaperPositionMode', 'auto');

	filename_out = strcat('artest_', short, '.png');

    export_fig(filename_out, '-painters', '-transparent', '-nocrop');
end

function [] = ultra2(data_sensor)
	hFig = figure;
	hold on;
	hTitle = title('Hovering Ultrasound Measurements');
	hXLabel = xlabel('Value (mm)');
	hYLabel = ylabel('Number of Readings');
	bins = 10;

	data_dirty = abs(data_sensor(55:154, 5));
	data_dirty = data_dirty(isfinite(data_dirty(:, 1)), :);
	data_clean = removeoutliers(data_dirty);

	% hold on;
	data_mean = mean(data_clean);
	data_std = std(data_clean);
	disp_text = sprintf( 'Standard Deviation: %3.3f\n', data_std);
	
	hHist = histfit(data_clean, bins);

	hText = text(1925, 12, disp_text);

	set(hHist(1),'facecolor',[168/255, 191/255, 183/255],...
				'edgecolor', [42/255, 69/255, 79/255],...
				'linewidth', 1); 

	set(hHist(2),'color', [42/255, 69/255, 79/255],...
				'linewidth', 3);

	set(gca,...
    'FontName', 'Helvetica');

    set([hTitle, hXLabel, hYLabel], ...
    'FontName', 'AvantGarde');

    set( hTitle, ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

	     set(gca, ...
	  'Box'         , 'off'     , ...
	  'TickDir'     , 'out'     , ...
	  'TickLength'  , [.02 .02] , ...
	  'XMinorTick'  , 'on'      , ...
	  'YMinorTick'  , 'on'      , ...
	  'ZMinorTick'  , 'on'      , ...
	  'XColor'      , [.3 .3 .3], ...
	  'YColor'      , [.3 .3 .3], ...
	  'ZColor'      , [.3 .3 .3], ...
	  'LineWidth'   , 1         );

    set(gcf, 'PaperPositionMode', 'auto');


    export_fig('ultra_test2.png', '-painters', '-transparent', '-nocrop');

	hold off;

end

function [] = ultra1(data_sensor)
	hFig = figure;
	hold on;
	hTitle = title('Hovering Ultrasound Measurements');
	hXLabel = xlabel('Value (mm)');
	hYLabel = ylabel('Number of Readings');
	bins = 10;

	data_dirty = abs(data_sensor(19:180, 5));
	data_dirty = data_dirty(isfinite(data_dirty(:, 1)), :);
	data_clean = removeoutliers(data_dirty);

	% hold on;
	data_mean = mean(data_clean);
	data_std = std(data_clean);
	disp_text = sprintf( 'Standard Deviation: %3.3f\n', data_std);
	
	hHist = histfit(data_clean, bins);

	hText = text(715, 25, disp_text);

	set(hHist(1),'facecolor',[168/255, 191/255, 183/255],...
				'edgecolor', [42/255, 69/255, 79/255],...
				'linewidth', 1); 

	set(hHist(2),'color', [42/255, 69/255, 79/255],...
				'linewidth', 3);

	set(gca,...
    'FontName', 'Helvetica');

    set([hTitle, hXLabel, hYLabel], ...
    'FontName', 'AvantGarde');

    set( hTitle, ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

	     set(gca, ...
	  'Box'         , 'off'     , ...
	  'TickDir'     , 'out'     , ...
	  'TickLength'  , [.02 .02] , ...
	  'XMinorTick'  , 'on'      , ...
	  'YMinorTick'  , 'on'      , ...
	  'ZMinorTick'  , 'on'      , ...
	  'XColor'      , [.3 .3 .3], ...
	  'YColor'      , [.3 .3 .3], ...
	  'ZColor'      , [.3 .3 .3], ...
	  'LineWidth'   , 1         );

    set(gcf, 'PaperPositionMode', 'auto');

    export_fig('ultra_test1.png', '-painters', '-transparent', '-nocrop');

	hold off;

end


function [] = gyr(data_sensor)
	% self.step, delta_t, x_vel, y_vel, altd, rotX, rotY, rotZ
	hFig = figure;
	hold on;
	hTitle = title('Stationary Gyroscope Measurements');
	hXLabel = xlabel('Value (degrees)');
	hYLabel = ylabel('Number of Readings');
	bins = 10;

	data_dirty = abs(data_sensor(:, 8));
	data_dirty = data_dirty(isfinite(data_dirty(:, 1)), :);
	data_clean = removeoutliers(data_dirty);

	% hold on;
	data_mean = mean(data_clean);
	data_std = std(data_clean);
	disp_text = sprintf( 'Standard Deviation: %3.3f\n', data_std);

	data_clean = data_clean - data_mean;
	
	hHist = histfit(data_clean, bins);

	hText = text(-17, 50, disp_text);

	set(hHist(1),'facecolor',[168/255, 191/255, 183/255],...
				'edgecolor', [42/255, 69/255, 79/255],...
				'linewidth', 1); 

	set(hHist(2),'color', [42/255, 69/255, 79/255],...
				'linewidth', 3);

	set(gca,...
    'FontName', 'Helvetica');

    set([hTitle, hXLabel, hYLabel], ...
    'FontName', 'AvantGarde');

    set( hTitle, ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

	     set(gca, ...
	  'Box'         , 'off'     , ...
	  'TickDir'     , 'out'     , ...
	  'TickLength'  , [.02 .02] , ...
	  'XMinorTick'  , 'on'      , ...
	  'YMinorTick'  , 'on'      , ...
	  'ZMinorTick'  , 'on'      , ...
	  'XColor'      , [.3 .3 .3], ...
	  'YColor'      , [.3 .3 .3], ...
	  'ZColor'      , [.3 .3 .3], ...
	  'LineWidth'   , 1         );

    set(gcf, 'PaperPositionMode', 'auto');

    export_fig('gyr_test.png', '-painters', '-transparent', '-nocrop');

	hold off;

end

function [] = prism()
	hFig = figure;
	hold on;
	grid on;
	axis equal;
	hTitle = title('Tag Detection Space');
	hXLabel = xlabel('X Position (mm)');
	hYLabel = ylabel('Y Position (mm)');
	hZLabel = zlabel('Z Position (mm)');
	view([131 32]);

	a = 1410;
	b= 880;
	y = [0 0 0 0; -a -a  a  a;  a -a -a  a];
	x = [0 0 0 0; -b  b  b  b; -b -b  b -b];
	z = [300 300 300 300; 3500 3500 3500 3500; 3500 3500 3500 3500];
	hCone = fill3(x,y,z, [0/255, 108/255, 128/255]);

	a = 135/2;
	b = 135/2;
	x = [-a; -a; a; a];
	y = [-b;  b; b; -b];
	z = [0; 0; 0; 0];

	hTag = fill3(x,y,z, 'k');
	hold off;

	set(hCone,...
		'EdgeColor', [19/255, 37/255, 55/255],...
		'LineWidth', 2);

	set(gca,...
    'FontName', 'Helvetica');

    set([hTitle, hXLabel, hYLabel, hZLabel], ...
    'FontName', 'AvantGarde');

    set( hTitle, ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

	     set(gca, ...
	  'Box'         , 'off'     , ...
	  'TickDir'     , 'out'     , ...
	  'TickLength'  , [.02 .02] , ...
	  'XMinorTick'  , 'on'      , ...
	  'YMinorTick'  , 'on'      , ...
	  'ZMinorTick'  , 'on'      , ...
	  'XColor'      , [.3 .3 .3], ...
	  'YColor'      , [.3 .3 .3], ...
	  'ZColor'      , [.3 .3 .3], ...
	  'LineWidth'   , 1         );

    set(gcf, 'PaperPositionMode', 'auto');

    export_fig('tag_detection.png', '-painters', '-transparent', '-nocrop');


end

function [] = particles(data_ar, data_alt, data_part, percentage)
	last_val = data_part(end, 1);

	num = round((percentage*length(data_alt))/100);
	disp(num);
	hFig = figure(1);
	set(hFig, 'Position', [0 0 800 600])

	hold on;
	hTitle = title('Particle Filter Estimated Position');
	hXLabel = xlabel('X Position (mm)');
	hYLabel = ylabel('Y Position (mm)');
	hZLabel = zlabel('Z Position (mm)');
	axis([-1000  4500 -1000 3500 0 2000 0 100]);
	view([-72 30]);
	axis equal;
	grid on;
	% x_avg = [];
	% y_avg = [];
	% x_std = [];
	% y_std = [];
	% for i=1:last_val
	% 	current_step = data_part(data_part(:, 1) == i, :);
	% 	x_avg(i) = mean(current_step(:, 3));
	% 	y_avg(i) = mean(current_step(:, 4));
	% 	x_std(i) = std(current_step(:, 3));
	% 	y_std(i) = std(current_step(:, 4));
	% 	if (mod(i, 1) == 0)
	% 		% scatter3(current_step(:, 3), current_step(:, 4), current_step(:, 5));
	% 		% scatter(current_step(:, 3), current_step(:, 4));
	% 	end
	% end

	tag_loc = [0, 0, 0; 3500, 0 , 0; 3500, 2500 ,0; 0, 2500, 0];
	
	data_alt_part = data_alt(data_alt(:, 1) < num, :);
	data_ar_part = data_ar(data_ar(:, 1) < num, :);

	hVis = plot3(data_alt_part(:, 7), data_alt_part(:, 8), data_alt_part(:, 9));
	hPF = plot3(data_alt_part(:, 3), data_alt_part(:, 4), data_alt_part(:, 5));
	hAR = plot3(data_ar_part(:, 3), data_ar_part(:, 4), data_ar_part(:, 5));
	hTag = plot3(tag_loc(:, 1), tag_loc(:, 2), tag_loc(:, 3));

	set(hVis,...
		'LineStyle', '--',...
		'LineWidth', 4,...
		'Color', [.8 0 0]);
	set(hPF,...
		'LineWidth', 4,...
		'Color', [0 0 .8]);
	set(hAR,...
		'LineStyle', 'none',...
		'Marker', 'o',...
		'MarkerSize', 6,...
		'MarkerFaceColor', [0 .5 0],...
		'MarkerEdgeColor', [0 .2 0]);
	set(hTag,...
		'LineStyle', 'none',...
		'Marker', 's',...
		'MarkerSize', 9,...
		'MarkerFaceColor', [.2 .2 .2],...
		'MarkerEdgeColor', [0 0 0]);

	hLegend = legend( ...
		[hVis, hPF, hAR, hTag], ...
		'Visual Odometry' , ...
		'Particle Filter'      , ...
		'AR Tag Estimated Position'       , ...
		'AR Tag Location'    , ...
		'location', 'NorthEast' );

	set(gca,...
    'FontName', 'Helvetica');

    set([hTitle, hXLabel, hYLabel, hZLabel], ...
    'FontName', 'AvantGarde');

    set([hLegend, gca],...
    'FontSize', 10);
    set([hXLabel, hYLabel, hZLabel],...
    'FontSize', 12);

    set( hTitle, ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

    set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'ZMinorTick'  , 'on'      , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , [.3 .3 .3], ...
  'ZColor'      , [.3 .3 .3], ...
  'LineWidth'   , 1         );

    set(gcf, 'PaperPositionMode', 'auto');


    filename_out = strcat('3dgraph_', num2str(percentage), '.png');
	export_fig(filename_out, '-painters', '-transparent', '-nocrop'); %-painters -nocrop
	
	
end

function [] = ar_pos(data_ar)
	figure;
	hold on;
	title('AR Tag Position')
	axis equal;
	plot3(data_ar(:, 3), data_ar(:, 4), data_ar(:, 5));
	hold off;
end

function [] = alt_pos(data_alt)
	figure;
	hold on;
	title('Combined alt position')
	axis equal;
	% plot3(data_alt(:, 3), data_alt(:, 4), data_alt(:, 5));
	plot(data_alt(:, 3), data_alt(:, 4));
	% for i=1:length(data_alt(:, 6))
	% 	x_width = 10%data_alt(i, 6) + .1
	% 	y_width = 10%data_alt(i, 7) + .1
	% 	rectangle('Position', [data_alt(i, 3) - x_width/2, data_alt(i, 4) - y_width/2, x_width, y_width])
	% end
	hold off;
end

function [] = rotation_values(data)
	figure;
	hold on;
	title('X Rotation');
	xlabel('Iteration');
	ylabel('Rotation (degrees)');
	plot(data(:, 1), data(:, 7));
	hold off;

	figure;
	hold on;
	title('Y Rotation');
	xlabel('Iteration');
	ylabel('Rotation (degrees)');
	plot(data(:, 1), data(:, 8));
	hold off;

	figure;
	hold on;
	title('Z Rotation');
	xlabel('Iteration');
	ylabel('Rotation (degrees)');
	plot(data(:, 1), data(:, 6));
	hold off;
end

function [] = vis_readings(data)
	figure;
	hold on;
	title('Visual Odometry');
	xlabel('Iteration');
	ylabel('X Velocity (m/s)');
	plot(data(:, 1), data(:, 14)/1000);
	hold off;

	figure;
	hold on;
	title('Visual Odometry');
	xlabel('Iteration');
	ylabel('Y Velocity (m/s)');
	plot(data(:, 1), data(:, 15)/1000);
	hold off;
end

function [ acc ] = convert_g(g)
	acc = g * 9806.65;
end

function [] = plot_acc(data)
	figure;
	hold on;
	title('X Acceleration (m/s/s)');
	xlabel('Iteration');
	ylabel('X Acceleration (m/s/s)');
	plot(data(:, 1), convert_g(data(:, 3) - mean(data(1:100, 3))));
	hold off;

	figure;
	hold on;
	title('Y Acceleration (m/s/s)');
	xlabel('Iteration');
	ylabel('Y Acceleration (m/s/s)');
	plot(data(:, 1), convert_g(data(:, 4) - mean(data(1:100, 4))));
	hold off;

	figure;
	hold on;
	title('Z Acceleration (m/s/s)');
	xlabel('Iteration');
	ylabel('Z Acceleration (m/s/s)');
	plot(data(:, 1), convert_g(data(:, 5) - mean(data(1:100, 5))));
	hold off;
end

function [] = acc_pos(data)
	figure;
	hold on;
	title('Accelerometer Position (m)');
	axis equal;
	xlabel('X Position (m)');
	ylabel('Y Position (m)');
	zlabel('Z Position (m)');
	plot3(data(:, 10)/1000, data(:, 11)/1000, data(:, 12)/1000);
	hold off;
end

function [] = vis_pos(data)
	figure;
	hold on;
	plot(data(:, 16)/1000);

	hold off;

	figure;
	hold on;
	title('Visual Odometry Position (m)');
	axis equal;
	xlabel('X Position (m)');
	ylabel('Y Position (m)');
	zlabel('Z Position (m)');
	line('XData',data(1, 23)/1000, 'YData',data(1, 24)/1000, 'ZData', data(1, 16)/1000, 'Color','g', 'marker','.', 'MarkerSize',50);
	line('XData',data(end, 23)/1000, 'YData',data(end, 24)/1000, 'ZData', data(end, 16)/1000, 'Color','r', 'marker','.', 'MarkerSize',50);
	plot3(data(:, 23)/1000, data(:, 24)/1000, data(:, 16)/1000);
	hold off;
end

function [] = sensor_data(data)
	% Sensor data modeling
	% draw_hist('X Acceleration', data, 3, 40);
	% draw_hist('Y Acceleration', data, 4, 40);
	% draw_hist('Z Acceleration', data, 5, 40);
	draw_hist('Gyroscope Theta', data, 6, 40);
	% draw_hist('X Magnetometer', data, 17, 10);
	% draw_hist('Y Magnetometer', data, 18, 10);
	% draw_hist('Magnetometer Theta Est', abs(data), 20, 20);
end

function [m_out] = rotate(m, rotX, rotY, rotZ)
	rotX_mat = [1, 0, 0, 0;
				0, cosd(rotX), sind(rotX), 0;
				0, -sind(rotX), cosd(rotX), 0;
				0, 0, 0, 1];
	rotY_mat = [cosd(rotY), 0, sind(rotY), 0;
				0, 1, 0, 0;
				-sind(rotY), 0, cosd(rotY), 0;
				0, 0, 0, 1];
	rotZ_mat = [cosd(rotZ), sind(rotZ), 0, 0;
				-sind(rotZ), cosd(rotZ), 0, 0;
				0, 0, 1, 0;
				0, 0, 0, 1];
	m_out = rotX_mat*rotY_mat*rotZ_mat*m;
	% m_out = rotZ_mat*rotY_mat*rotX_mat*m;
end

% So this is in the local coordinate frame.....
function [] = euler_method(data)
	vel_acc = zeros([1, 3]);
	pos_acc = zeros([1, 3]);
	pos_vis = zeros([1, 3]);

	global_acc = zeros([1, 4]);

	time_sum(1) = 0;
	for i=2:length(data(:, 3))
		acc_m = [convert_g(data(i, 3)); convert_g(data(i, 4)); convert_g(data(i, 5)); 1];
		rotX = data(i, 7) - mean(data(1:10, 7));
		rotY = data(i, 8) - mean(data(1:10, 8));
		rotZ = data(i, 6) - mean(data(1:10, 6));
		global_acc(i, :) = rotate(acc_m, rotX, rotY, rotZ);

		% global_acc(i, 3) = global_acc(i, 3) - convert_g(mean(data(1:10, 5)));
		
		global_acc(i, 1) = global_acc(i, 1) - convert_g(mean(data(1:10, 3)));
		global_acc(i, 2) = global_acc(i, 2) - convert_g(mean(data(1:10, 4)));
		global_acc(i, 3) = global_acc(i, 3) - convert_g(mean(data(1:10, 5)));

		for j=1:3
			vel_acc(i, j) = vel_acc(i-1, j) + global_acc(i, j)*data(i,2);
			pos_acc(i, j) = pos_acc(i-1, j) + vel_acc(i, j)*data(i,2);
			time_sum(i) = time_sum(i-1) + data(i, 2);
		end
	end

	figure;
	hold on;
	title('Global X Acceleration (m/s/s)');
	xlabel('Time (s)');
	ylabel('X Acc (m/s/s)');
	plot(time_sum(2:end), global_acc(2:end, 1)/1000);
	hold off;

	figure;
	hold on;
	title('Global Y Acceleration (m/s/s)');
	xlabel('Time (s)');
	ylabel('Y Acc (m/s/s)');
	plot(time_sum(2:end), global_acc(2:end, 2)/1000);
	hold off;

	figure;
	hold on;
	title('Global Z Acceleration (m/s/s)');
	xlabel('Time (s)');
	ylabel('Z Acc (m/s/s)');
	plot(time_sum(2:end), global_acc(2:end, 3)/1000);
	hold off;

	figure;
	hold on;
	title('Accelerometer Euler Method X Velocity (m/s)');
	xlabel('Time (s)');
	ylabel('X Vel (m/s)');
	plot(time_sum(:), vel_acc(:, 1)/1000);
	hold off;

	figure;
	hold on;
	title('Accelerometer Euler Method Y Velocity (m/s)');
	xlabel('Time (s)');
	ylabel('Y Vel (m/s)');
	plot(time_sum(:), vel_acc(:, 2)/1000);
	hold off;

	figure;
	hold on;
	title('Accelerometer Euler Method Z Velocity (m/s)');
	xlabel('Time (s)');
	ylabel('Z Vel (m/s)');
	plot(time_sum(:), vel_acc(:, 3)/1000);
	hold off;


	figure;
	hold on;
	title('Accelerometer Euler Method Position (m)');
	axis equal;
	xlabel('X Position (m)');
	ylabel('Y Position (m)');
	zlabel('Z Position (m)');
	plot3(pos_acc(:, 1)/1000, pos_acc(:, 2)/1000, pos_acc(:, 3)/1000);
	hold off;

	% figure;
	% hold on;
	% title('Visual Odometry Euler Method Position (m)');
	% xlabel('X Position (m)');
	% ylabel('Y Position (m)');
	% plot(pos(:, 1)/1000, pos(:, 2)/1000);
	% hold off;
end

function [] = draw_hist(graph_title, data, col, bins)
	figure;
	hold on;
	title(graph_title);
	data_dirty = data(:, col);
	data_dirty = data_dirty(isfinite(data_dirty(:, 1)), :);
	data_clean = removeoutliers(data_dirty);

	% hold on;
	data_mean = mean(data_clean);
	data_std = std(data_clean);
	disp(sprintf( 'mean: %f', data_mean));
	disp(sprintf( 'stdev: %f\n\n',data_std));
	histfit(data_clean, bins);

	hold off;
end
