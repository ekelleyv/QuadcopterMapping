function [] = plot_path(filename)
%PLOT_PATH Summary of this function goes here
%   Detailed explanation goes here
	[time, x, y, z, alt] = importfile(filename)
	
%	plot(x, y)
	grid on
	plot3(x, y, alt)
%    plot(time, alt)
end

