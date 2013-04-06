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
	data = data_obj.data;

	% acc_pos(data);
	% sensor_data(data);
end

function [] = acc_pos(data)
	figure
	hold on
	title('Accelerometer Position (m)');
	xlabel('X Position (m)');
	ylabel('Y Position (m)');
	zlabel('Z Position (m)')
	plot3(data(:, 10)/1000, data(:, 11)/1000, data(:, 12)/1000);
	hold off
end

function [] = sensor_data(data)
	% Sensor data modeling
	% draw_hist('X Acceleration', data, 3, 40);
	% draw_hist('Y Acceleration', data, 4, 40);
	% draw_hist('Z Acceleration', data, 5, 40);
	% draw_hist('Gyroscope Theta', data, 6, 40);
	% draw_hist('X Magnetometer', data, 17, 10);
	% draw_hist('Y Magnetometer', data, 18, 10);
	% draw_hist('Magnetometer Theta Est', abs(data), 20, 20);
end

function [] = euler_method(data)
	vel(1, 1) = 0;
	vel(1, 2) = 0;
	vel(1, 3) = 0;
	
	pos(1, 1) = 0;
	pos(1, 2) = 0;
	pos(1, 3) = 0;

	time_sum(1) = 0;
	for i=2:length(data(:, 3))
		for j=1:3
			vel(i, j) = vel(i-1, j) + data(i, 2+j)*data(i, 2);
			pos(i, j) = pos(i-1, j) + vel(i-1, j)*data(i,2);
			time_sum(i) = time_sum(i-1) + data(i, 2);
		end
	end

	figure;
	hold on;
	title('Euler Method Position (m)');
	xlabel('X Position (m)');
	ylabel('Y Position (m)');
	plot(pos(:, 1)/1000, pos(:, 2)/1000);
	hold off;
end

function [] = draw_hist(graph_title, data, col, bins)
	figure
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
