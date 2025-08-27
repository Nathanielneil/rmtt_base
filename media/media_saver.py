import os
import shutil
from datetime import datetime
from pathlib import Path
from utils.logger import Logger
from config.settings import *


class MediaSaver:
    def __init__(self):
        self.logger = Logger("MediaSaver")
        self._ensure_directories()
    
    def _ensure_directories(self):
        IMAGES_DIR.mkdir(parents=True, exist_ok=True)
        VIDEOS_DIR.mkdir(parents=True, exist_ok=True)
        
    def get_media_stats(self):
        try:
            images = list(IMAGES_DIR.glob("*.jpg")) + list(IMAGES_DIR.glob("*.png"))
            videos = list(VIDEOS_DIR.glob("*.avi")) + list(VIDEOS_DIR.glob("*.mp4"))
            
            image_size = sum(f.stat().st_size for f in images)
            video_size = sum(f.stat().st_size for f in videos)
            
            stats = {
                'images': {
                    'count': len(images),
                    'size_mb': round(image_size / (1024*1024), 2)
                },
                'videos': {
                    'count': len(videos),
                    'size_mb': round(video_size / (1024*1024), 2)
                },
                'total_size_mb': round((image_size + video_size) / (1024*1024), 2)
            }
            
            return stats
            
        except Exception as e:
            self.logger.error(f"获取媒体统计失败: {str(e)}")
            return None
    
    def list_images(self, limit=None):
        try:
            images = sorted(
                IMAGES_DIR.glob("*.jpg"), 
                key=lambda x: x.stat().st_mtime, 
                reverse=True
            )
            
            if limit:
                images = images[:limit]
            
            return [str(img) for img in images]
            
        except Exception as e:
            self.logger.error(f"列出图片失败: {str(e)}")
            return []
    
    def list_videos(self, limit=None):
        try:
            videos = sorted(
                VIDEOS_DIR.glob("*.avi"), 
                key=lambda x: x.stat().st_mtime, 
                reverse=True
            )
            
            if limit:
                videos = videos[:limit]
            
            return [str(vid) for vid in videos]
            
        except Exception as e:
            self.logger.error(f"列出视频失败: {str(e)}")
            return []
    
    def delete_media(self, filepath):
        try:
            path = Path(filepath)
            if path.exists():
                path.unlink()
                self.logger.info(f"已删除文件: {filepath}")
                return True
            else:
                self.logger.warning(f"文件不存在: {filepath}")
                return False
                
        except Exception as e:
            self.logger.error(f"删除文件失败: {str(e)}")
            return False
    
    def cleanup_old_media(self, days=7):
        try:
            import time
            cutoff_time = time.time() - (days * 24 * 60 * 60)
            deleted_count = 0
            
            for filepath in list(IMAGES_DIR.iterdir()) + list(VIDEOS_DIR.iterdir()):
                if filepath.stat().st_mtime < cutoff_time:
                    filepath.unlink()
                    deleted_count += 1
            
            self.logger.info(f"清理了 {deleted_count} 个超过 {days} 天的媒体文件")
            return deleted_count
            
        except Exception as e:
            self.logger.error(f"清理媒体文件失败: {str(e)}")
            return 0
    
    def export_media(self, destination_path):
        try:
            dest_path = Path(destination_path)
            dest_path.mkdir(parents=True, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            export_dir = dest_path / f"tello_media_export_{timestamp}"
            export_dir.mkdir()
            
            images_export = export_dir / "images"
            videos_export = export_dir / "videos"
            
            shutil.copytree(IMAGES_DIR, images_export, dirs_exist_ok=True)
            shutil.copytree(VIDEOS_DIR, videos_export, dirs_exist_ok=True)
            
            self.logger.info(f"媒体文件已导出到: {export_dir}")
            return str(export_dir)
            
        except Exception as e:
            self.logger.error(f"导出媒体文件失败: {str(e)}")
            return None
    
    def create_media_backup(self):
        backup_name = f"media_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.zip"
        backup_path = DATA_DIR / backup_name
        
        try:
            import zipfile
            
            with zipfile.ZipFile(backup_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for root, dirs, files in os.walk(IMAGES_DIR):
                    for file in files:
                        file_path = Path(root) / file
                        arc_path = Path("images") / file_path.relative_to(IMAGES_DIR)
                        zipf.write(file_path, arc_path)
                
                for root, dirs, files in os.walk(VIDEOS_DIR):
                    for file in files:
                        file_path = Path(root) / file
                        arc_path = Path("videos") / file_path.relative_to(VIDEOS_DIR)
                        zipf.write(file_path, arc_path)
            
            self.logger.info(f"媒体备份已创建: {backup_path}")
            return str(backup_path)
            
        except Exception as e:
            self.logger.error(f"创建备份失败: {str(e)}")
            return None