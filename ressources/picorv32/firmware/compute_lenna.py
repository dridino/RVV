from PIL import Image

img = Image.open("lenna_32_32.png").convert("RGB")
width, height = img.size
pixels = img.load()

for i in range(height):
    for j in range(width):
        print("r[" + str(i*width + j) + "] = " + str(pixels[j, i][0]) + ";")
        print("g[" + str(i*width + j) + "] = " + str(pixels[j, i][1]) + ";")
        print("b[" + str(i*width + j) + "] = " + str(pixels[j, i][2]) + ";")