import cv2
import cv2.aruco as aruco
import numpy as np
from fpdf import FPDF

# Parameters for the grid
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # Corrected function
marker_size = 100  # Size of each marker in pixels
grid_rows = 5  # Number of rows in the grid
grid_cols = 4  # Number of columns in the grid
marker_spacing = 10  # Spacing between markers in pixels

# Calculate grid image size
grid_width = grid_cols * (marker_size + marker_spacing) - marker_spacing
grid_height = grid_rows * (marker_size + marker_spacing) - marker_spacing

# Create a blank white image for the grid
grid_image = np.ones((grid_height, grid_width), dtype=np.uint8) * 255

# Populate the grid with markers
marker_id = 0
for row in range(grid_rows):
    for col in range(grid_cols):
        if marker_id >= 50:  # Stop if we exceed the dictionary size
            break
        # Generate the marker
        marker = aruco.generateImageMarker(dictionary, marker_id, marker_size)
        # Calculate the position in the grid
        y_start = row * (marker_size + marker_spacing)
        x_start = col * (marker_size + marker_spacing)
        # Place the marker on the grid
        grid_image[y_start:y_start + marker_size, x_start:x_start + marker_size] = marker
        marker_id += 1

# Save the grid image temporarily
temp_image_file = "temp_aruco_grid.png"
cv2.imwrite(temp_image_file, grid_image)

# Create a PDF and add the image
pdf = FPDF(orientation='P', unit='mm', format='Letter')  # Letter format for 8.5x11 inches
pdf.add_page()

# Scale the image to fit the Letter page width
letter_width_mm = 215.9  # Letter width in mm
letter_height_mm = 279.4  # Letter height in mm
img_width_mm = letter_width_mm - 20  # Leave 10mm margin on each side
img_height_mm = img_width_mm * (grid_image.shape[0] / grid_image.shape[1])  # Maintain aspect ratio

# Add the image to the PDF
pdf.image(temp_image_file, x=10, y=10, w=img_width_mm, h=img_height_mm)

# Save the PDF
output_pdf_filename = "aruco_marker_grid.pdf"
pdf.output(output_pdf_filename)

# Cleanup temporary image file
import os
os.remove(temp_image_file)

print(f"Grid of markers saved as {output_pdf_filename}.")
