import cv2
import threading
import time
from datetime import datetime
from pathlib import Path
from utils.logger import Logger
from utils.exceptions import TelloConnectionError
from config.settings import *


class VideoStreamHandler:
    def __init__(self, connection_manager):
        self.logger = Logger("VideoStreamHandler")
        self.connection_manager = connection_manager
        self.streaming = False
        self.recording = False
        self._stream_thread = None
        self._record_thread = None
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.video_writer = None
        self.record_filename = None
    
    def start_stream(self):
        try:
            if not self.connection_manager.is_connected():
                raise TelloConnectionError("未连接到Tello，无法启动视频流")
            
            if self.streaming:
                self.logger.warning("视频流已经在运行")
                return
            
            tello = self.connection_manager.get_tello()
            tello.streamon()
            
            self.streaming = True
            self._stream_thread = threading.Thread(target=self._stream_worker)
            self._stream_thread.daemon = True
            self._stream_thread.start()
            
            self.logger.info("视频流已启动")
            
        except Exception as e:
            self.logger.error(f"启动视频流失败: {str(e)}")
            raise
    
    def stop_stream(self):
        try:
            if not self.streaming:
                return
            
            self.streaming = False
            
            if self.recording:
                self.stop_recording()
            
            if self._stream_thread:
                self._stream_thread.join(timeout=2)
            
            if self.connection_manager.is_connected():
                tello = self.connection_manager.get_tello()
                tello.streamoff()
            
            self.logger.info("视频流已停止")
            
        except Exception as e:
            self.logger.error(f"停止视频流失败: {str(e)}")
    
    def _stream_worker(self):
        tello = self.connection_manager.get_tello()
        
        while self.streaming:
            try:
                frame = tello.get_frame_read().frame
                if frame is not None:
                    with self.frame_lock:
                        self.current_frame = frame.copy()
                
                time.sleep(1/VIDEO_FPS)
                
            except Exception as e:
                self.logger.error(f"视频流处理出错: {str(e)}")
                time.sleep(0.1)
    
    def get_current_frame(self):
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None
    
    def start_recording(self, filename=None):
        try:
            if not self.streaming:
                raise Exception("必须先启动视频流才能录制")
            
            if self.recording:
                self.logger.warning("正在录制中")
                return
            
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"tello_video_{timestamp}.avi"
            
            self.record_filename = VIDEOS_DIR / filename
            VIDEOS_DIR.mkdir(parents=True, exist_ok=True)
            
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(
                str(self.record_filename), 
                fourcc, 
                VIDEO_FPS, 
                VIDEO_RESOLUTION
            )
            
            self.recording = True
            self._record_thread = threading.Thread(target=self._record_worker)
            self._record_thread.daemon = True
            self._record_thread.start()
            
            self.logger.info(f"开始录制视频: {filename}")
            
        except Exception as e:
            self.logger.error(f"开始录制失败: {str(e)}")
            raise
    
    def stop_recording(self):
        try:
            if not self.recording:
                return
            
            self.recording = False
            
            if self._record_thread:
                self._record_thread.join(timeout=2)
            
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            
            if self.record_filename:
                self.logger.info(f"录制完成，文件保存至: {self.record_filename}")
                self.record_filename = None
            
        except Exception as e:
            self.logger.error(f"停止录制失败: {str(e)}")
    
    def _record_worker(self):
        while self.recording and self.streaming:
            try:
                frame = self.get_current_frame()
                if frame is not None and self.video_writer:
                    resized_frame = cv2.resize(frame, VIDEO_RESOLUTION)
                    self.video_writer.write(resized_frame)
                
                time.sleep(1/VIDEO_FPS)
                
            except Exception as e:
                self.logger.error(f"录制过程出错: {str(e)}")
                time.sleep(0.1)
    
    def capture_image(self, filename=None):
        try:
            frame = self.get_current_frame()
            if frame is None:
                raise Exception("无法获取当前帧")
            
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"tello_image_{timestamp}.jpg"
            
            image_path = IMAGES_DIR / filename
            IMAGES_DIR.mkdir(parents=True, exist_ok=True)
            
            cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, IMAGE_QUALITY])
            
            self.logger.info(f"图片已保存: {image_path}")
            return str(image_path)
            
        except Exception as e:
            self.logger.error(f"拍照失败: {str(e)}")
            raise