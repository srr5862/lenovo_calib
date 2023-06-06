import matplotlib.pyplot as plt


img_path = "./debug/230529T110630S397/230529T110638S706_image.jpg"
depth_path = "./debug/230529T110630S397/230529T110638S862_re_depth.png"

depth = plt.imread(depth_path)
rgb = plt.imread(img_path)


plt.imshow(depth,alpha=0.9)
plt.imshow(rgb,alpha=0.6)
plt.show()