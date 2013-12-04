import sys

openfile = open(sys.argv[1])
blk = -1
newline = []
for line in openfile:
	tmp = line.rstrip().rsplit(",")
	_blk = int(tmp[3])

	if _blk != blk:
		if blk != -1:
			print ",".join(newline)
		newline = [tmp[1], tmp[2]]
		blk = _blk;

		for i in range(8):
			try:
				newline.append(tmp[9+2*i])
			except:
				print line

openfile.close()
