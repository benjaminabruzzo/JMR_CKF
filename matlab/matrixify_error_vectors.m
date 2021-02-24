function A = matrixify_error_vectors(A, B)
	% A = batch_data.(matlab.lang.makeValidName(meta.exp_code)).evt.(matlab.lang.makeValidName(sprintf( 'map_%s', batch_data.(matlab.lang.makeValidName(meta.exp_code)).jstr{idx}(1)))).ugv1.ckf.p.rms_2D;
	% B = errors.clip.ugv1.ckf.p.rms_2D';
	try % just adding assuming the vectors are the same length
		A = [A;B];
	catch % hey, they're not the same length, do something to fix it
		% if the new data has a longer vector, it needs to be flipped, trimmed and concatenated
		if (size(B,2) > size(A,2)) % then there are more elements in the new data than the old
			offset = size(B,2) - size(A,2)+1;
			A = [A; B(1,offset:end)];
		else % by logic size() < size()
			offset = size(A,2) - size(B,2)+1;
			A = [A(:,offset:end); B];
		end
	end
end
