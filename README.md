# SHAPE-DETECTION-USING-DE2-115
This project detects and classifies simple geometric shapes in live video, entirely in hardware. An OV7670 camera feeds a Terasic DE2-115 board (Cyclone IV E), and the design is written in Verilog with no processor or software. The camera image is stored in a 320x240 frame buffer, and a hardware pipeline separates the object from the background, measures its edge directions, and classifies it as a circle, square, rectangle, triangle or ellipse. The result is drawn as a bounding box over the VGA picture and printed on the board’s 16x2 LCD.

The detector measures the background colour from the picture edge every frame, so it works on black, white or coloured backgrounds and under changing light. It identifies shapes from the direction pattern of their edges, which does not change when the object rotates, so tilted shapes (including 45 degrees) are recognised. It also handles partly visible objects, and the bounding box follows the tilt of the object. A moment-based test recognises any kind of triangle.

Developed with Quartus Prime and ModelSim as part of an M.Tech VLSI Design project.
