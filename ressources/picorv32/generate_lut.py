size = 8
name = "vcpop_lut"

def vcpop(i):
    return bin(i)[2:].count('1')

func = vcpop

print(f"function [{size-1}:0] {name};")
print(f"    input [{size-1}:0] addr;")
print("    begin")
print("        case (addr)")
for i in range(2**size):
    print(f"            {size}'h{hex(i)[2:]}: {name} = {size}'h{hex(func(i))[2:]};")
print("        endcase")
print("    end")
print("endfunction")