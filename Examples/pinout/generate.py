# modified code from icopins (icoboard)
# for blackicemx by Hirosh Dabui
#!/usr/bin/python3

database = list()

with open("blackice-mx.pcf", "r") as f:
    for line in f:
        line = line.split()
        if len(line) >= 3 and line[0] == "set_io" and line[1] != "clk":
            database.append([line[2], line[1], []])

mem_width = len(database)
mem_depth = 256

def rs232_byte(value):
    bits = []

    # start bit
    bits.append(False)

    # data bits (LSB first)
    for i in range(8):
        bits.append((value & (1 << i)) != 0)

    # stop bit
    bits.append(True)

    return bits


for pad, net, bits in database:
    msg = "%s: %s\r\n" % (net, pad)

    for c in msg:
        bits += rs232_byte(ord(c))

    # add idle period
    while len(bits) < mem_depth:
        bits.append(True)

    assert len(bits) == mem_depth

with open("defines.h", "w") as f:
    print("`define OUTPUT_PINS \\", file=f)
    for pad, net, bits in database:
        print("output %s, \\" % net, file=f)
    print("", file=f)

    print("`define OUTPUT_EXPR {%s}" % ", ".join([net for pad, net, bits in database]), file=f)
    print("`define MEM_WIDTH %d" % mem_width, file=f)
    print("`define MEM_DEPTH %d" % mem_depth, file=f)

mem_data = []
for i in range(mem_depth):
    mem_data.append(["0"] * mem_width)

for i in range(len(database)):
    for j in range(mem_depth):
        mem_data[j][i] = "1" if database[i][2][j] else "0"

with open("memdata.dat", "w") as f:
    for word in mem_data:
        print("".join(word), file=f)

