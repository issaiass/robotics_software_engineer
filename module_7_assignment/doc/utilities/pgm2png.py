import os
from PIL import Image

src = 'module_7_assignment/map'
dst = 'module_7_assignment/doc/imgs'
for file in os.listdir(src):
    filename, extension  = os.path.splitext(file)
    if extension == ".pgm":
        new_file = "{}.png".format(filename)
        new_file = os.path.join(dst, new_file) 
        print(new_file)
        file = os.path.join(src, file)
        with Image.open(file) as im:    
            im.save(new_file)

