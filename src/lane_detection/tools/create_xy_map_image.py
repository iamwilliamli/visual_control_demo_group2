import numpy as np
from scipy.optimize import curve_fit
import cv2

pixel_xy_mapper = [
    [31, 293, 0.13, -0.1],
    [121, 289, 0.13, -0.05],
    [216, 287, 0.13, 0.0],
    [307, 292, 0.13, 0.05],
    [398, 297, 0.13, 0.1],
    [11, 220, 0.18, -0.15],
    [79, 221, 0.18, -0.1],
    [146, 222, 0.18, -0.05],
    [214, 220, 0.18, 0.0],
    [281, 218, 0.18, 0.05],
    [348, 219, 0.18, 0.1],
    [417, 222, 0.18, 0.15],
    [52, 176, 0.24, -0.15],
    [106, 178, 0.24, -0.1],
    [159, 179, 0.24, -0.05],
    [213, 178, 0.24, 0.0],
    [266, 179, 0.24, 0.05],
    [318, 177, 0.24, 0.1],
    [373, 175, 0.24, 0.15],
    [83, 143, 0.3, -0.15],
    [126, 144, 0.3, -0.1],
    [169, 145, 0.3, -0.05],
    [211, 144, 0.3, 0.0],
    [254, 145, 0.3, 0.05],
    [297, 147, 0.3, 0.1],
    [339, 147, 0.3, 0.15],
    [115, 113, 0.4, -0.15],
    [146, 112, 0.4, -0.1],
    [178, 113, 0.4, -0.05],
    [211, 113, 0.4, 0.0],
    [242, 114, 0.4, 0.05],
    [274, 112, 0.4, 0.1],
    [305, 111, 0.4, 0.15],
    [134, 93, 0.5, -0.15],
    [160, 92, 0.5, -0.1],
    [185, 92, 0.5, -0.05],
    [210, 93, 0.5, 0.0],
    [235, 92, 0.5, 0.05],
    [261, 92, 0.5, 0.1],
    [286, 92, 0.5, 0.15],
    [312, 92, 0.5, 0.2],
    [124, 80, 0.6, -0.2],
    [146, 81, 0.6, -0.15],
    [167, 81, 0.6, -0.1],
    [189, 81, 0.6, -0.05],
    [210, 80, 0.6, 0.0],
    [230, 81, 0.6, 0.05],
    [252, 80, 0.6, 0.1],
    [272, 82, 0.6, 0.15],
    [294, 81, 0.6, 0.2],
]

X_SIZE = 439
Y_SIZE = 313

for i in range(len(pixel_xy_mapper)):
    pixel_xy_mapper[i][1] = Y_SIZE - pixel_xy_mapper[i][1]
    
# Prepare data
pixel_xy_mapper = np.array(pixel_xy_mapper)
px = pixel_xy_mapper[:, 0]
py = pixel_xy_mapper[:, 1]
x = pixel_xy_mapper[:, 2]
y = pixel_xy_mapper[:, 3]

# Fit x_model
def x_func(py, a, b, c, d, e):
    return a + b * py + c * np.exp(d * (py - e))

popt_x, _ = curve_fit(x_func, py, x, maxfev=10000)

# Fit y_model
def y_func(pxy, f, g, h, i, k):
    px, py = pxy
    return f * (px - g) / (h - i * py) + k

popt_y, _ = curve_fit(y_func, (px, py), y, maxfev=10000)

print("x_model params: a, b, c, d, e =", popt_x)
print("y_model params: f, g, h, i, k =", popt_y)

# Create empty image with 3 channels (float32 for precision)
xy_map_img = np.zeros((Y_SIZE, X_SIZE, 3), dtype=np.float32)

# Fill the image with [x, y, 0] values derived from the models
for py_img in range(Y_SIZE):
    if py_img > 270:
        continue
    
    for px_img in range(X_SIZE):
        # Compute x using the fitted x_model (depends only on py)
        x_val = x_func(py_img, *popt_x)
        # Compute y using the fitted y_model (depends on px, py)
        y_val = y_func((px_img, py_img), *popt_y)
        xy_map_img[Y_SIZE - py_img - 1, px_img, 0] = x_val
        xy_map_img[Y_SIZE - py_img - 1, px_img, 1] = y_val
        xy_map_img[Y_SIZE - py_img - 1, px_img, 2] = 0.0  # Placeholder for third channel

# Optionally, save the image as a .npy file for later use
# Save as .npy file for later use
np.save("../data/xy_map_image.npy", xy_map_img)

# Normalize x and y channels to [0, 255] for visualization and save as PNG
x_norm = cv2.normalize(xy_map_img[:, :, 0], None, 0, 255, cv2.NORM_MINMAX)
y_norm = cv2.normalize(xy_map_img[:, :, 1], None, 0, 255, cv2.NORM_MINMAX)
img_to_save = np.stack([x_norm, y_norm, np.zeros_like(x_norm)], axis=2).astype(np.uint8)
cv2.imwrite("../data/xy_map_image.png", img_to_save)


