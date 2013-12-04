import sys
import matplotlib.pyplot as plt

seqno = []
openfile = open(sys.argv[1], "r")
for line in openfile:
	tmp = line.rstrip().rsplit(",")
	seqno.append(int(tmp[5]))
	print line
openfile.close()

plt.plot(seqno, "-x")
plt.grid(True)
plt.show()
