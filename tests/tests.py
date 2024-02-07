import cv2
import numpy as np

# Load the image from a file
image_path = '../images/light_source.jpg'  # Replace with the actual path to your image file
image = cv2.imread(image_path)

# Check if the image is loaded successfully
if image is None:
    print(f"Error: Unable to load the image at '{image_path}'")
    exit()

# Convert the image from BGR to RGB (OpenCV uses BGR by default)
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
image_rgb_sum = np.sum(image_rgb, axis=2, keepdims=True)
cv2.imshow("Image", image_rgb)
cv2.waitKey(0)
cv2.destroyAllWindows()
pix = np.sum(image_rgb[0][0])
sum_pix = image_rgb_sum[0][0]


print(pix)
print(sum_pix)
