import cv2
import numpy as np

def abs_sobel_thresh(image, orient='x', sobel_kernel=3, thresh=(0, 255)):
    """
    Apply SobelX or SobelY operator to the image with the specified kernel and threshold
    """    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grad = cv2.Sobel(gray, cv2.CV_64F, 1 if orient == 'x' else 0, 1 if orient == 'y' else 0, ksize=sobel_kernel)
    grad = np.abs(grad)
    max_grad = np.max(grad)
    grad = np.uint8(grad * 255 / max_grad)
    mask = np.zeros_like(grad, np.float32)
    mask[(grad >= thresh[0]) & (grad <= thresh[1])] = 1
    
    return mask

def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    """
    Apply both SobelX or SobelY operators with the specified kernel
    to the image  and use magnitude threshold
    """    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    mag = np.sqrt(grad_x * grad_x + grad_y * grad_y)
    max_mag = np.max(mag)
    mag = np.uint8(mag * 255 / max_mag)
    mask = np.zeros_like(mag, np.float32)
    mask[(mag >= mag_thresh[0]) & (mag <= mag_thresh[1])] = 1
    
    return mask

def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi/2)):
    """
    Apply both SobelX or SobelY operator with the specified kernel to the image, 
    determine gradient direction and use direction threshold
    """    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    
    grad_x = np.abs(grad_x)
    grad_y = np.abs(grad_y)
        
    dir = np.arctan2(grad_y, grad_x)
    
    mask = np.zeros_like(dir)
    mask[(dir >= thresh[0]) & (dir <= thresh[1])] = 1
    
    return mask

def hls_select(image, thresh=(0, 255)):
    """
    Convert image to HLS and select pixels based on Saturation threshold
    """    
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    s = hls[:,:,2]
    mask = np.zeros_like(s)
    mask[(s >= thresh[0]) & (s <= thresh[1])] = 1
    return mask

def pipline(image):
    combined = abs_sobel_thresh(image, orient='x', sobel_kernel=15, thresh=(35, 255))
    
    hls_s = hls_select(image, thresh=(120, 255))

    out_img = np.dstack((hls_s, hls_s, hls_s)) * 255

    hls_s = abs_sobel_thresh(out_img, orient='x', sobel_kernel=15, thresh=(50, 255))
 
    combined_res = np.zeros_like(hls_s)
    
    combined_res[((hls_s == 1) | (combined == 1))] = 1
       
    out_img = np.dstack((combined_res, combined_res, combined_res)) * 255

    return out_img