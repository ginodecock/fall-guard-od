from PIL import Image
import numpy as np

def png_to_argb4444_uchar_array_with_black_bg(image_path):
    img = Image.open(image_path).convert('RGBA')
    width, height = img.size
    data = np.array(img)

    def to_4bit(value):
        return (value >> 4) & 0xF

    uchar_data = []
    for y in range(height):
        for x in range(width):
            r, g, b, a = data[y, x]
            # Set RGB to black if alpha is fully transparent
            if a == 0:
                r, g, b = 0, 0, 0
            a4 = to_4bit(a)
            r4 = to_4bit(r)
            g4 = to_4bit(g)
            b4 = to_4bit(b)
            argb4444 = (a4 << 12) | (r4 << 8) | (g4 << 4) | b4
            high_byte = (argb4444 >> 8) & 0xFF
            low_byte = argb4444 & 0xFF
            uchar_data.append(high_byte)
            uchar_data.append(low_byte)

    c_array = f"const unsigned char image_data[{len(uchar_data)}] = {{\n"
    line = ""
    for i, val in enumerate(uchar_data):
        line += f"0x{val:02X}, "
        if (i + 1) % 16 == 0:
            c_array += line + "\n"
            line = ""
    if line:
        c_array += line + "\n"
    c_array += "};"

    return c_array

# Example usage:
# print(png_to_argb4444_uchar_array_with_black_bg('your_image.png'))


print(png_to_argb4444_uchar_array_with_black_bg('g-dc75.png'))
