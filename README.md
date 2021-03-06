# Real time implementation of multispectral fusion

The primary focus of this project is the fusion of multispectral images which was acquired by two distinctly different cameras (Different Specifications i.e., resolution, Zoom, Grayscale or color). This project was inspired from the SDMSI (Software defined Multi Spectral Imaging for Artic Sensor Networks)  paper by Prof. Sam Siewert et al. A real time fusion system has been designed to meet its deadline. 
The system was realized in NVIDIA Jetson TK1 embedded Development Kit. Target system is developed to detect unmanned aerial vehicles/flights, threats in coastal borders.  During the absence of light, and when the potential threat is camouflaged with the environment; it is very hard to find the threats using Visible cameras. LWIR (Long wave infrared cameras) which is of 8-15 µm wavelength can detect the heat waves emitted by the threat. This character of LWIR can be used with Visible image captured from Visible camera to identify potential threat. 

# Implementation
Implementation is briefly described in the LWIRVisibleFusion.pdf.

Implemented using 

1. pthreads

2. OpenCV

3. Video Resources from online
