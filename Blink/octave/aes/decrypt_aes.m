function [pos1 val1 pos2 val2]=decrypt_aes(input)

s=aesinit([0:15]);
in = uint16(input);
in = typecast(in,'uint8');
long_in = zeros(1,length(in));
for i=1:length(in)
    long_in(1,i) = in(i);
end
out = aes(s, 'dec', 'ecb', long_in);

for i=1:16
    short_out(1,i) = uint8(out(1,i));
end

out = typecast(short_out, 'single');
pos1 = uint32(out(1,1));
val1 = single(out(1,2));
pos2 = uint32(out(1,3));
val2 = single(out(1,4));
end
