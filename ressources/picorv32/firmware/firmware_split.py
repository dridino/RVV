# Open `firmware.hex` which is a list of words, one per line
# and split it in 4 different `.mif` files, one for each byte of each word
# By doing so, each firmware[0-3].mif file contains every byte n°[0-3] of
# each word, one byte per line

with open("firmware.hex", 'r') as firmware:
    l = [open("firmware0.mif", 'w'), open("firmware1.mif", 'w'), open("firmware2.mif", 'w'), open("firmware3.mif", 'w')]

    for i in range(len(l)):
        l[i].write("WIDTH=8;\n")
        l[i].write("DEPTH=32768;\n")
        l[i].write("ADDRESS_RADIX=HEX;\n")
        l[i].write("DATA_RADIX=HEX;\n")
        l[i].write("CONTENT BEGIN\n")
    cpt = 0
    while True:
        line = firmware.readline()
        if line == "": break
        if len(line) < 8: line = "00000000"
        for i in range(len(l)):
            l[i].write(format(cpt, '08x') + " : " + line[6-(2*i):8-(2*i)] + ";\n")
        cpt+=1
    
    for i in range(len(l)):
        l[i].write("END;")
        l[i].close()
