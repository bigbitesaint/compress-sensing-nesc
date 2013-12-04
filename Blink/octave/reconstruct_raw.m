function reconstruct_raw(N)
	filename = '../script/raw.csv';
	true_s = ones(N,1)*100;


	% each packet entry is 2 byte, and we are supposed to work with 4 byte data
	% so for M record, we need to get N*4 bytes, which equals to N*4/2 entries
	% each packet contains 16 data
	row_needed = (N*2)/16;

	data=csvread(filename);
	% get number of cols
	col=size(data,2);
	data=data([1:row_needed],[2:col]);
	y = reshape( data', N*2, 1 );
	y = uint16(y);
	y = typecast(y,'uint32')

	err = single(0);
	for i=1:N
		if abs(true_s(i,1)) > eps
			err = err + abs(true_s(i,1)-y(i,1))/abs(true_s(i,1))
		end
	end
	fprintf('Distortion: %f\n', (single(err)/N));
end
