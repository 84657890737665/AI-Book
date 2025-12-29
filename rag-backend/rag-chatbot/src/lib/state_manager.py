"""Processing state management for the RAG Pipeline."""
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any
from .utils import save_json, load_json


class ProcessingState:
    """Manages the state of the ingestion pipeline to enable resuming after interruptions."""
    
    def __init__(self, pipeline_id: str, state_file: str = None):
        """
        Initialize the processing state manager.
        
        Args:
            pipeline_id: Unique identifier for this pipeline run
            state_file: Optional path to the state file (default: ./output/{pipeline_id}_state.json)
        """
        self.pipeline_id = pipeline_id
        self.state_file = state_file or f"./output/{pipeline_id}_state.json"
        self.state = self._load_state()
    
    def _load_state(self) -> Dict[str, Any]:
        """Load the processing state from file, or create a default state."""
        state_path = Path(self.state_file)
        
        if state_path.exists():
            try:
                return load_json(self.state_file)
            except (json.JSONDecodeError, FileNotFoundError):
                # If there's an error loading the state file, start fresh
                pass
        
        # Create a default state
        return {
            "id": self.pipeline_id,
            "pipeline_id": self.pipeline_id,
            "urls_processed": [],
            "urls_remaining": [],
            "current_status": "idle",
            "last_updated": datetime.now().isoformat(),
            "error_info": None
        }
    
    def save_state(self) -> None:
        """Save the current processing state to file."""
        self.state["last_updated"] = datetime.now().isoformat()
        save_json(self.state, self.state_file)
    
    def update_status(self, status: str) -> None:
        """
        Update the current status of the pipeline.
        
        Args:
            status: The new status (e.g., 'idle', 'running', 'paused', 'error')
        """
        self.state["current_status"] = status
        self.save_state()
    
    def add_processed_url(self, url: str) -> None:
        """
        Add a URL to the list of processed URLs.
        
        Args:
            url: The URL that was processed
        """
        if url not in self.state["urls_processed"]:
            self.state["urls_processed"].append(url)
        
        # Remove from remaining URLs if it's there
        if url in self.state["urls_remaining"]:
            self.state["urls_remaining"].remove(url)
        
        self.save_state()
    
    def set_remaining_urls(self, urls: List[str]) -> None:
        """
        Set the list of remaining URLs to process.
        
        Args:
            urls: List of URLs that still need to be processed
        """
        self.state["urls_remaining"] = urls
        self.save_state()
    
    def get_remaining_urls(self) -> List[str]:
        """
        Get the list of remaining URLs to process.
        
        Returns:
            List of URLs that still need to be processed
        """
        return self.state["urls_remaining"]
    
    def get_processed_urls(self) -> List[str]:
        """
        Get the list of already processed URLs.
        
        Returns:
            List of URLs that have been processed
        """
        return self.state["urls_processed"]
    
    def has_remaining_urls(self) -> bool:
        """
        Check if there are any remaining URLs to process.
        
        Returns:
            True if there are remaining URLs, False otherwise
        """
        return len(self.state["urls_remaining"]) > 0
    
    def set_error_info(self, error_info: Dict[str, Any]) -> None:
        """
        Set error information in the state.
        
        Args:
            error_info: Dictionary containing error details
        """
        self.state["error_info"] = error_info
        self.save_state()
    
    def clear_error_info(self) -> None:
        """Clear any error information from the state."""
        self.state["error_info"] = None
        self.save_state()
    
    def reset(self) -> None:
        """Reset the processing state to its initial values."""
        self.state = {
            "id": self.pipeline_id,
            "pipeline_id": self.pipeline_id,
            "urls_processed": [],
            "urls_remaining": [],
            "current_status": "idle",
            "last_updated": datetime.now().isoformat(),
            "error_info": None
        }
        self.save_state()