# Gate detection data from vision node
float32 error_x       # Normalized horizontal error [-1.0, 1.0], positive = gate is right of center
float32 error_y       # Normalized vertical error [-1.0, 1.0], positive = gate is below center
float32 width_ratio   # Bounding box width as fraction of frame width [0.0, 1.0]
bool gate_visible     # True if gate detected above confidence threshold
