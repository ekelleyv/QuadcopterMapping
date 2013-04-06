function [time, x, y, z, alt] = import(filename)
	fid = fopen(filename)
	data = csvread(filename, 1, 0)

