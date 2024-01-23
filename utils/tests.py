import cv2
import numpy as np

# Load the image from a file
image_path = '../images/500x500-0.02_pix.jpg'  # Replace with the actual path to your image file
image = cv2.imread(image_path)

# Check if the image is loaded successfully
if image is None:
    print(f"Error: Unable to load the image at '{image_path}'")
    exit()

# Convert the image from BGR to RGB (OpenCV uses BGR by default)
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
cv2.imshow("Image", image_rgb)
cv2.waitKey(0)
cv2.destroyAllWindows()
