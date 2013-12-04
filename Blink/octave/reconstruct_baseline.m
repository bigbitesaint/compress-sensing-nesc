function reconstruct_baseline(N, M)
	addpath('../../octave/aes');
	filename = 'baseline.csv';


	%reconstructed signal
	recon_s=[];

	%original signal
%	true_s=csvread('../script/ground.csv');
%	true_s=true_s(:,[2:size(true_s,2)-1]);
%	true_s=reshape(true_s', N*2, 1);
%	true_s=uint16(true_s);
%	true_s=typecast(true_s, 'uint32')

	%matrices
	[phi psi inv_psi] = generateMatrixA(N,M,3,2);



	data=csvread(filename);
	% get number of cols
	[rows cols] = size(data);
	cols = min(M*2+2, cols)

	% each packet entry is 2 byte, and we are supposed to work with 4 byte data
	% so for M record, we need to get M*4 bytes, which equals to M*4/2 entries
	row_needed = (M*2)/(cols-2)


	fp = fopen('baseline_result.csv', 'w');
	prev_i = 0;
	err = 0;
	for k=0:row_needed:rows-row_needed
		fprintf('Progress: [%d/%d]\n', k ,  rows-row_needed);
		pdata = data([k+1:k+row_needed],[3:cols]);
		pdata = reshape( pdata', M*2, 1 );
		tdata = int16(pdata);
		pdata = uint16(pdata);

		y = zeros(N,1);


		if M==2
			% aes-128
			for i=1:4:M*2
				if tdata(i,1) != -1
					p1 = typecast( pdata([i:i+1],1)', 'single' );
					v1 = typecast( pdata([i+2:i+3],1)', 'single' );
					%[p1 v1 p2 v2] = decrypt_aes( pdata([i:i+3], 1)' )

					if p1>=0 & p1 < N
						y(p1+1,1) = v1;
					end

				end
			end
		else
			% aes-128
			for i=1:8:M*2
				if tdata(i,1) != -1
					p1 = typecast( pdata([i:i+1],1)', 'single' );
					v1 = typecast( pdata([i+2:i+3],1)', 'single' );
					p2 = typecast( pdata([i+4:i+5],1)', 'single' );
					v2 = typecast( pdata([i+6:i+7],1)', 'single' );
					%[p1 v1 p2 v2] = decrypt_aes( pdata([i:i+7], 1)' )

					if p1>=0 & p1 < N
						y(p1+1,1) = v1;
					end

					if p2>=0 & p2 < N
						y(p2+1,1) = v2;
					end
				end
			end
		end

		s=psi*y;

		fprintf(fp,'%f,%f',data(k+1,1),data(k+1,2));
		for i=1:length(s)
			fprintf(fp,',%d',s(i,1));
		end
		fprintf(fp,'\n');
		
	%	recon_s = [ recon_s ; s ];
	end


%	for i=1:length(recon_s)
%		fprintf(fp,'%d, %d\n',i,recon_s(i,1));
%	end
%	fclose(fp);


%	for i=1:N
%		if abs(true_s(i,1)) > eps
%			err = err + single(abs(true_s(i,1)-s(i,1)))/single(abs(true_s(i,1)));
%		end
%	end
%	fprintf('Distortion: %f\n', err/ single(N*floor(row/row_needed)) );
end
