07-11-26 Added aruco_image_detector.py in erl1/erl1

# Remember to colcon build

# Addition of service to take photo
Ho commentato la parte di codice in [aruco_image_detector](assignment/assignment/aruco_image_detector.py) da line 110 a 133 che serviva a pubblicare l'immagine del marker modificata(cerchio+id) sul topic /marker_image, adesso l'operazione è eseguita all'interno del servizio [TakePhoto](assignment_interfaces/srv/TakePhoto.srv).
In particolare l'handler del servizio modifica l'immagine, fornita dal sub al topic /camera/image/compressed definito nello stesso nodo, nello stesso modo (cerchio+id) e la pusha su /marker_image.
Il servizio viene chiamato dalla [fsm](assignment/assignment/FSM.cpp) al nuovo stato TAKE_PHOTO (tra gli stati APPROACH ed EXECUTE), aspettando la risposta dell'handler del servizio prima di passare allo stato EXECUTE.
Ho aggiunto anche dei log all'inizio di ogni stato per debugguare, fornendo anche il valore di current_id.
Altri cambiamenti riguardano il launch file con aggiunta del nodo per il servizio e il CMakeLists di assignment_interfaces per includere il servizio.

Dai log in [fsm](assignment/assignment/FSM.cpp) si nota che dopo che il robot ruota davanti al marker 2, si può già vedere l'id del marker 4 ma anche di marker 5 e 20 che non esistono proprio, probabilmente frutto della fusione di più marker nell'immagine della camera allo stesso tempo -> source from web:
- question: is it possible that multiple aruco markers get fused in a simulation and the resulting marker is a different one for ex i get marker 5 when there were marker 2,3 and 4 (using the original dictionary of aruco)
- answer(from Gemini): Yes, it is definitely possible for multiple closely placed ArUco markers to visually merge and trigger a false positive detection for a completely different ID (such as ID 5), especially when using DICT_ARUCO_ORIGINAL.

Why This Happens

• Contour Merging at Distance or Low Resolution. The detector starts by thresholding the image and finding square contours. If markers 2, 3, and 4 are placed too close together without a sufficient white border ("quiet zone"), downsampling, anti-aliasing, or motion blur in your simulation can blend their black outer borders into a single large quadrilateral.
• Weak Error-Correction in DICT_ARUCO_ORIGINAL. Unlike newer OpenCV dictionaries (like DICT_6X6_250), DICT_ARUCO_ORIGINAL packs 1,024 markers into a small 5×5 grid. Because it uses nearly all available bit combinations (storing the ID in the 2nd and 4th columns with minimal parity bits), the Hamming distance between valid codes is very small. Almost any random 5×5 grid of black and white pixels sampled from a merged contour has a high statistical chance of matching some valid ID in that dictionary.
• Bit-Correction Tolerance. By default, OpenCV's errorCorrectionRate allows the detector to "fix" flipped bits to handle noise. When a merged blob forms a messy 5×5 pattern, the decoder can aggressively correct a few cells and misclassify the entire group as marker 5.

How to Prevent It in Simulation

• Switch Dictionaries. If you aren't strictly locked into DICT_ARUCO_ORIGINAL, switch to a dictionary with a larger minimum Hamming distance and fewer IDs, such as DICT_4X4_50 or DICT_6X6_50. This drastically lowers the odds of random patterns matching a valid ID.
• Enforce a White Quiet Zone. Ensure every marker in your simulation textures has a white margin around its black border that is at least 1 to 2 bit-widths wide so adjacent markers never bleed into a single contour.
• Tighten Detector Parameters. In OpenCV's DetectorParameters, lower errorCorrectionRate (e.g., from 0.6 down to 0.2 or 0.3) to make the decoder stricter, and tune minMarkerDistanceRate or maxMarkerPerimeterRate to reject oversized merged contours.
• Filter by Known IDs. In your application logic, maintain a whitelist of expected IDs (e.g., [2, 3, 4]) and discard any detection outside that list.