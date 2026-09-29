import cv2
import numpy as np
import time
from PIL import Image
import torch
import sys
sys.path.append('/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/src/perception/perception/BGNet')  # Add BGNet directory to sys.path
from datasets.data_io import get_transform
from models.bgnet_plus import BGNet_Plus

# ==========================================
# 1. Global Model Initialization
# (Run this ONCE before your HoloOcean loop)
# ==========================================
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Load BGNet
bgnet_model = BGNet_Plus().to(device)
checkpoint = torch.load('/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/src/perception/perception/BGNet/models/Sceneflow-IRS-BGNet-Plus.pth', map_location=device)
bgnet_model.load_state_dict(checkpoint)
bgnet_model.eval()

# Load preprocessing transforms
preprocess = get_transform()

# Safely handle PIL's deprecated ANTIALIAS based on version
try:
    resample_filter = Image.ANTIALIAS
except AttributeError:
    resample_filter = Image.Resampling.LANCZOS

    last_depth_timings = {}


    def get_last_depth_timings():
        """Return timings from the most recent depth computation in milliseconds."""
        return dict(last_depth_timings)


# ==========================================
# 2. The Upgraded Compute Function
# ==========================================
def compute_stereo_depth(left_img, right_img, focal_length_px, baseline_meters):
    """
    Computes depth map using BGNet_Plus.
    Expects left_img and right_img as standard OpenCV BGR numpy arrays.
    """
    started = time.perf_counter()

    # 1. Convert OpenCV BGR arrays to PIL Grayscale images
    left_pil = Image.fromarray(cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY))
    right_pil = Image.fromarray(cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY))
    
    orig_w, orig_h = left_pil.size
    
    # 2. Resize to nearest multiple of 64 (Required by BGNet)
    h1 = int(orig_h - (orig_h % 64))
    w1 = int(orig_w - (orig_w % 64))
    
    left_resized = left_pil.resize((w1, h1), resample_filter)
    right_resized = right_pil.resize((w1, h1), resample_filter)
    
    # 3. Convert to Float32 arrays and apply transforms
    left_np = np.ascontiguousarray(left_resized, dtype=np.float32)
    right_np = np.ascontiguousarray(right_resized, dtype=np.float32)
    
    left_tensor = preprocess(left_np).unsqueeze(0).to(device)
    right_tensor = preprocess(right_np).unsqueeze(0).to(device)
    preprocessing_ms = (time.perf_counter() - started) * 1000.0
    
    # 4. Neural Network Inference (No Gradients for speed and memory)
    if device.type == 'cuda':
        torch.cuda.synchronize(device)
    inference_started = time.perf_counter()
    with torch.no_grad():
        pred, _ = bgnet_model(left_tensor, right_tensor)
    if device.type == 'cuda':
        torch.cuda.synchronize(device)
    inference_ms = (time.perf_counter() - inference_started) * 1000.0
        
    # 5. Extract disparity map in pixels (Do NOT multiply by 256)
    postprocessing_started = time.perf_counter()
    disparity_resized = pred[0].data.cpu().numpy()
    
    # 6. Resize disparity back to original dimensions (e.g., 256x256 -> 288x288)
    # When upscaling the image, the pixel disparities must also be multiplied by the scale factor
    width_ratio = orig_w / w1
    disparity_orig = cv2.resize(disparity_resized, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
    disparity_orig = disparity_orig * width_ratio
    
    # 7. Convert Disparity to Depth (Meters)
    # Filter invalid disparities to prevent divide-by-zero
    disparity_orig[disparity_orig <= 0.0] = 0.1
    
    depth_map = (focal_length_px * baseline_meters) / disparity_orig
    
    # Mask out bad depth
    depth_map[disparity_orig == 0.1] = 0.0 

    h, w = depth_map.shape
    cy, cx = h // 2, w // 2
    
    # 2. Extract the metric depth at the center
    center_depth = depth_map[cy, cx]
    postprocessing_ms = (time.perf_counter() - postprocessing_started) * 1000.0
    global last_depth_timings
    last_depth_timings = {
        'preprocessing_ms': preprocessing_ms,
        'inference_ms': inference_ms,
        'postprocessing_ms': postprocessing_ms,
        'total_compute_ms': (time.perf_counter() - started) * 1000.0,
        'center_depth_m': float(center_depth),
    }
    
    return depth_map