"""Torch/OpenCV implementation. Model loading occurs only at construction."""
from pathlib import Path
import time
import cv2
import numpy as np
from PIL import Image
import torch
import torchvision.transforms as transforms
from ..BGNet.models.bgnet_plus import BGNet_Plus

class BGNetDisparityEstimator:
    def __init__(self, checkpoint_path=None, clock=time):
        self.clock = clock
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = BGNet_Plus().to(self.device)
        checkpoint_path = checkpoint_path or (
            Path(__file__).resolve().parents[1] / 'BGNet' / 'models' /
            'Sceneflow-IRS-BGNet-Plus.pth')
        checkpoint = torch.load(str(checkpoint_path), map_location=self.device)
        self.model.load_state_dict(checkpoint)
        self.model.eval()
        self.preprocess = transforms.Compose([transforms.ToTensor()])
        try:
            self.resample_filter = Image.ANTIALIAS
        except AttributeError:
            self.resample_filter = Image.Resampling.LANCZOS

    def estimate(self, left_img, right_img, started):
        # 1. Convert OpenCV BGR arrays to PIL Grayscale images
        left_pil = Image.fromarray(cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY))
        right_pil = Image.fromarray(cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY))

        orig_w, orig_h = left_pil.size

        # 2. Resize to nearest multiple of 64 (Required by BGNet)
        h1 = int(orig_h - (orig_h % 64))
        w1 = int(orig_w - (orig_w % 64))

        left_resized = left_pil.resize((w1, h1), self.resample_filter)
        right_resized = right_pil.resize((w1, h1), self.resample_filter)

        # 3. Convert to Float32 arrays and apply transforms
        left_np = np.ascontiguousarray(left_resized, dtype=np.float32)
        right_np = np.ascontiguousarray(right_resized, dtype=np.float32)

        left_tensor = self.preprocess(left_np).unsqueeze(0).to(self.device)
        right_tensor = self.preprocess(right_np).unsqueeze(0).to(self.device)
        preprocessing_ms = (self.clock.perf_counter() - started) * 1000.0

        # 4. Neural Network Inference (No Gradients for speed and memory)
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)
        inference_started = self.clock.perf_counter()
        with torch.no_grad():
            pred, _ = self.model(left_tensor, right_tensor)
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)
        inference_ms = (self.clock.perf_counter() - inference_started) * 1000.0

        # 5. Extract disparity map in pixels (Do NOT multiply by 256)
        postprocessing_started = self.clock.perf_counter()
        disparity_resized = pred[0].data.cpu().numpy()

        # 6. Resize disparity back to original dimensions (e.g., 256x256 -> 288x288)
        # When upscaling the image, the pixel disparities must also be multiplied by the scale factor
        width_ratio = orig_w / w1
        disparity_orig = cv2.resize(disparity_resized, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        disparity_orig = disparity_orig * width_ratio


        return disparity_orig, preprocessing_ms, inference_ms, postprocessing_started
