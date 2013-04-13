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


function [ data_obj ] = generate_figures( filename )
	data_obj = importdata(filename, ',', 1);

	ar_filename = strcat(filename(1:end-4), '_ar.txt')
	alt_filename = strcat(filename(1:end-4), '_alt.txt')
	data_ar = importdata(ar_filename, ',');
	data_alt = importdata(alt_filename, ',');

	% data = data_obj.data;
	% data = data(2:end, :);
	% % acc_pos(data);
	% vis_pos(data);
	% vis_readings(data);
	% sensor_data(data);
	% euler_method(data);
	% plot_acc(data);
	% rotation_values(data);

	ar_pos(data_ar);
	alt_pos(data_alt);




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
	for i=1:length(data_alt(:, 6))
		x_width = 10%data_alt(i, 6) + .1
		y_width = 10%data_alt(i, 7) + .1
		rectangle('Position', [data_alt(i, 3) - x_width/2, data_alt(i, 4) - y_width/2, x_width, y_width])
	end
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
	draw_hist('X Acceleration', data, 3, 40);
	draw_hist('Y Acceleration', data, 4, 40);
	draw_hist('Z Acceleration', data, 5, 40);
	draw_hist('Gyroscope Theta', data, 6, 40);
	draw_hist('X Magnetometer', data, 17, 10);
	draw_hist('Y Magnetometer', data, 18, 10);
	draw_hist('Magnetometer Theta Est', abs(data), 20, 20);
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
