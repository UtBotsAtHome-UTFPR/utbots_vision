import numpy as np
from collections import deque, Counter

class TargetBox:
    def __init__(self, target_id, x_min, y_min, x_max, y_max):
        self.id = target_id
        # We simplify by keeping only the 3D center (Z=0) for vector math
        self.center = np.array([(x_min + x_max) // 2, (y_min + y_max) // 2, 0])

class Person:
    def __init__(self, person_id):
        self.id = person_id
        self.state = "SEARCHING"  
        
        # Buffer for FFT (60 frames = ~2 seconds @ 30fps)
        self.angle_history = deque(maxlen=60) 
        
        self.origin = None 
        self.endpoint = None

    def update_pose(self, origin_pt, endpoint_pt, target_list):
        # Arm points:
        self.origin = np.array(origin_pt)
        self.endpoint = np.array(endpoint_pt)
        
        if self.state == "SEARCHING":
            self._check_waving()
            return None 
            
        elif self.state == "LOCKED":
            return self._process_pointing(target_list)

    def _check_waving(self):
        # Checks arm oscillation using FFT:

        vector = self.endpoint - self.origin
        dx, dy, _ = vector

        # In the image, Y grows downwards. Negative dy = wrist above the elbow
        if dy >= 0: 
            self.angle_history.clear()
            return

        angle = np.degrees(np.arctan2(-dy, dx))
        self.angle_history.append(angle)

        if len(self.angle_history) < self.angle_history.maxlen:
            return

        data = np.array(self.angle_history)
        
        # Rest filter (saves CPU if the arm is still)
        if np.std(data) < 8.0: 
            return

        # Centers the data and applies the Fourier Transform
        data_centered = data - np.mean(data)
        yf = np.fft.rfft(data_centered)
        xf = np.fft.rfftfreq(len(data), 1.0 / 30.0) 
        
        magnitude = np.abs(yf)

        # Finds the dominant frequency (ignoring 0Hz)
        peak_idx = np.argmax(magnitude[1:]) + 1 
        dominant_freq = xf[peak_idx]
        peak_mag = magnitude[peak_idx]

        # Trigger: If it matches the characteristics of a human wave
        if 1.0 <= dominant_freq <= 4.0 and peak_mag > 100:
            self.state = "LOCKED" 
            print(f"\n[FSM] Operator Locked via FFT! (Freq: {dominant_freq:.1f}Hz, Force: {peak_mag:.0f})\n")

    def _process_pointing(self, target_list):
        # Crosses the arm vector with bounding boxes that are in a dynamic list
        vector = self.endpoint - self.origin
        
        # Pointing must be directed downwards
        if vector[1] <= 0: return None
        
        norm = np.linalg.norm(vector)
        if norm == 0: return None
        
        min_dist = float('inf')
        target_id = None

        for target in target_list:
            target_vector = target.center - self.origin
            # Dot product to ensure the target is in front of the vector
            if np.dot(vector, target_vector) > 0:  
                # Perpendicular distance 
                dist = np.linalg.norm(np.cross(vector, target_vector)) / norm
                if dist < min_dist:
                    min_dist = dist
                    target_id = target.id
                    
        return target_id

class TemporalFilter:
    def __init__(self, buffer_size=15, min_votes=8):
        self.buffer = deque(maxlen=buffer_size)
        self.min_votes = min_votes

    def update(self, current_target):
        """Ensures we won't send false positives to the BT"""
        self.buffer.append(current_target)
        if len(self.buffer) < self.min_votes: return None
        
        most_voted, votes = Counter(self.buffer).most_common(1)[0]
        if votes >= self.min_votes and most_voted is not None:
            return most_voted
        return None